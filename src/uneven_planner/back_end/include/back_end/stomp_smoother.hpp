//
// Created by mazheng on 25-5-14.
//

#ifndef SRC_STOMP_SMOOTHER_HPP
#define SRC_STOMP_SMOOTHER_HPP

#include "back_end/stomp.hpp"
#include <ros/ros.h>
#include <Eigen/Geometry>
#include <utils/math_util.hpp>

namespace uneven_planner {

    /** @brief A dummy task for testing STOMP */
    class DummyTask : public Task
    {
    public:
        /**
         * @brief A dummy task for testing Stomp
         * @param parameters_bias default parameter bias used for computing cost for the test
         * @param bias_thresholds threshold to determine whether two trajectories are equal
         * @param std_dev standard deviation used for generating noisy parameters
         */
        DummyTask(const Eigen::MatrixXd& parameters_bias,
                  const std::vector<double>& bias_thresholds,
                  const std::vector<double>& std_dev, const Eigen::Vector3f& pose, const Eigen::Quaterniond& q)
                : parameters_bias_(parameters_bias), bias_thresholds_(bias_thresholds), std_dev_(std_dev), cur_pose_(pose), q_(q)
        {
            // generate smoothing matrix
            int num_timesteps = parameters_bias.cols();
            generateSmoothingMatrix(num_timesteps, 1.0, smoothing_M_);
            srand(1);
            param_.mass = 20.0f;
            param_.cgHeight = 0.25;
            param_.g = 9.81;
            param_.rearWeightFraction = 0.7;
            param_.trackWidth = 0.48;
            param_.wheelbase = 0.45;
        }

        /** @brief See base clase for documentation */
        bool generateNoisyParameters(const Eigen::MatrixXd& parameters,
                                     std::size_t start_timestep,
                                     std::size_t num_timesteps,
                                     int iteration_number,
                                     int rollout_number,
                                     Eigen::MatrixXd& parameters_noise,
                                     Eigen::MatrixXd& noise) override
        {
            double rand_noise;
            for (std::size_t d = 0; d < parameters.rows(); d++)
            {
                for (std::size_t t = 0; t < parameters.cols(); t++)
                {
                    rand_noise = static_cast<double>(rand() % RAND_MAX) / static_cast<double>(RAND_MAX - 1);  // 0 to 1
                    rand_noise = 2 * (0.5 - rand_noise);
                    noise(d, t) = rand_noise * std_dev_[d];
                }
            }

            parameters_noise = parameters + noise;

            return true;
        }

        bool computeCosts(const Eigen::MatrixXd& parameters,
                          std::size_t start_timestep,
                          std::size_t num_timesteps,
                          int iteration_number,
                          Eigen::VectorXd& costs,
                          bool& validity) override
        {
            return computeNoisyCosts(parameters, start_timestep, num_timesteps, iteration_number, -1, costs, validity);
        }

        bool computeNoisyCosts(const Eigen::MatrixXd& parameters,
                               std::size_t start_timestep,
                               std::size_t num_timesteps,
                               int iteration_number,
                               int rollout_number,
                               Eigen::VectorXd& costs,
                               bool& validity) override
        {
            costs.setZero(num_timesteps);
            double diff;
            validity = true;


            for (std::size_t t = 0u; t < num_timesteps; t++)
            {
                for (std::size_t d = 0u; d < parameters.rows(); d++)
                {
                    diff = std::abs(parameters(d, t) - parameters_bias_(d, t));
                    if (diff > std::abs(bias_thresholds_[d]))
                    {
                        validity = false;
                    }
                }
            }
            auto path = matrixToPath(parameters);
            for (auto point : path) {
                std::cout << "debug compute cost path: " << point.x() << " " << point.y() << " " << point.z() << std::endl;
            }
            double theta_slope, psi_s;
            computeSlopeAngles(q_, param_.g, cur_pose_[2], theta_slope, psi_s);
            auto max_angular_vel = 0.5;
            for (int j = 1; j < path.size(); ++j) {
                double spline_cost = 0;
                auto last_position = path.at(j - 1);
                auto pos = path.at(j);
                Eigen::Vector3f pose_diff = path.at(j) - path.at(j-1);
                float connection_angle = atan2(pose_diff.y(), pose_diff.x());
                float rotate_sum = abs(wrapToPi(connection_angle - path.at(j - 1)[2]))
                                   + abs(wrapToPi(path.at(j)[2] - connection_angle));
                bool is_reverse = false;
                if (rotate_sum < M_PI) {
                    // Forward
                    is_reverse = false;
                } else {
                    // Backward
                    is_reverse = true;
                }
                auto max_linear_vel = is_reverse ? 0.5 : 0.2;
                /// calculate time
                auto dist_cost = (pos.head(2) - last_position.head(2)).norm() / max_linear_vel;
                auto angle_cost =
                        std::fabs(wrapToPi(pos(2) - last_position(2))) / max_angular_vel;
                spline_cost += std::fmax(dist_cost, angle_cost);
                std::cout << "debug index: " << j << " time cost: " << spline_cost << std::endl;
                /// force penalty
                double pitch, roll;
                auto cur_pose = last_position;
                auto next_pose = pos;
                auto delta_theta = wrapToPi(next_pose[2] - cur_pose[2]);
                auto length = (next_pose - cur_pose).head(2).norm();
                double psi_i = cur_pose[2];
                computePointAttitude(theta_slope, psi_s, psi_i, pitch, roll);
                double N_l, N_r;
                computeForcesImproved(pitch, roll, N_l, N_r, param_);
                const double mu = 1.0f;
                auto F_l = mu * N_l;
                auto F_r = mu * N_r;
                auto dir = is_reverse ? -1 : 1;
                auto d_r = dir * length + param_.wheelbase / 2 * delta_theta;
                auto d_l = dir * length - param_.wheelbase / 2 * delta_theta;
                if (length < 1e-6 && std::fabs(delta_theta) < 1e-6) {
                    continue;
                }
                auto w_drive = F_l * std::fabs(d_l) + F_r * std::fabs(d_r);
                auto a = tan(theta_slope) * cos(psi_s);
                auto b = tan(theta_slope) * sin(psi_s);
                auto delta_z = a * (next_pose.x() - cur_pose.x()) + b * (next_pose.y() - cur_pose.y())
                               + 0.09 * (a * (cos(next_pose.z()) - cos(cur_pose.z())) + b * (sin(next_pose.z()) - sin(cur_pose.z())));
                auto w_grav = param_.mass * param_.g * delta_z;
                auto delta_w = w_drive;
                auto cost = 1 / delta_w;
                spline_cost += cost * 0;
                std::cout << "debug current: " << cur_pose[0] << " " << cur_pose[1] << " " << cur_pose[2] / M_PI * 180
                          << " to next point: " << next_pose[0] << " " << next_pose[1] << " " << next_pose[2] / M_PI * 180
                          << " path dist: " << length << " theta diff: " << delta_theta << " pitch: "
                          << pitch / M_PI * 180 << " roll: " << roll / M_PI * 180 << " and forces: " << N_l << "," << N_r
                          << " force: " << F_l << " " << F_r << " dist: " << d_l << " " << d_r << " work drive: " << w_drive
                          << " delta z: " << delta_z << " work grav: " << w_grav << " cost: " << cost << "total: " << spline_cost << std::endl;
                costs(j) = spline_cost;
            }

            return true;
        }

        bool filterParameterUpdates(std::size_t start_timestep,
                                    std::size_t num_timesteps,
                                    int iteration_number,
                                    const Eigen::MatrixXd& parameters,
                                    Eigen::MatrixXd& updates) override
        {
            return smoothParameterUpdates(start_timestep, num_timesteps, iteration_number, updates);
        }
    protected:
        /**
         * @brief Perform a smooth update given a noisy update
         * @param start_timestep starting timestep
         * @param num_timesteps number of timesteps
         * @param iteration_number number of interations allowed
         * @param updates returned smooth update
         * @return True if successful, otherwise false
         */
        bool smoothParameterUpdates(std::size_t start_timestep,
                                    std::size_t num_timesteps,
                                    int iteration_number,
                                    Eigen::MatrixXd& updates)
        {
            for (auto d = 0u; d < updates.rows(); d++)
            {
                updates.row(d).transpose() = smoothing_M_ * (updates.row(d).transpose());
            }

            return true;
        }
        static void computeSlopeAngles(const Eigen::Quaterniond& q, double g, double yaw,
                                       double& theta_slope, double& psi_s) {
            // 1. 构造机体->世界旋转矩阵（四元数转旋转矩阵）
            Eigen::Matrix3d Rwb = q.toRotationMatrix();  // :contentReference[oaicite:4]{index=4}

            // 2. 世界重力 [0,0,-g] 投影到机体坐标系
            Eigen::Vector3d g_body = Rwb.transpose() * Eigen::Vector3d(0.0, 0.0, -g);  //

            // 分量
            double gx = g_body.x();
            double gy = g_body.y();
            double gz = g_body.z();

            // 3. 计算坡度角
            theta_slope = std::atan2(std::hypot(gx, gy), std::abs(gz));  //

            // 4. 机体坐标系下坡面上升方向
            double psi_body = std::atan2(-gy, -gx);  // :contentReference[oaicite:5]{index=5}

            // 5. 规范化输入 yaw 到 (-π, π]
            double yaw_n =
                    std::atan2(std::sin(yaw), std::cos(yaw));  // :contentReference[oaicite:6]{index=6}

            // 6. 全局系下坡度方向角 = yaw_n + psi_body，再规范化
            double raw = yaw_n + psi_body;
            psi_s =
                    std::atan2(std::sin(raw), std::cos(raw));  // :contentReference[oaicite:7]{index=7}
        }

        static void computePointAttitude(double theta_slope, double psi_s, double psi_i, double &pitch, double &roll) {
            double delta_psi = psi_s - psi_i;  // 修正方向差顺序
            delta_psi = std::atan2(std::sin(delta_psi), std::cos(delta_psi));
            // 计算俯仰角
            pitch = -theta_slope * std::cos(delta_psi);
            // 计算横滚角
            roll = theta_slope * std::sin(delta_psi);
        }

        static void computeForcesImproved(double pitch, double roll, double& N_L, double& N_R, const VehicleParams& params) {
            // 重力分解（车辆姿态）：
            double g_x = params.g * sin(pitch);  // 纵向分量
            double g_y = params.g * sin(roll);  // 横向分量
            double g_z = params.g * cos(pitch) * cos(roll);  // 垂直分量

            // 静态后轴法向力（后轮）：
            double N0 = (params.mass * g_z * params.rearWeightFraction) / 2.0;

            // 载荷转移：
            // 纵向载荷转移（由俯仰角引起）
            double deltaN_long =
                    - (params.mass * params.g * sin(pitch) * params.cgHeight) / params.wheelbase;
            // 横向载荷转移（由横滚角引起）
            double deltaN_roll =
                    (params.mass * params.g * sin(roll) * params.cgHeight) / params.trackWidth;
            // 侧向加速度引起的载荷转移
            double deltaN_total = deltaN_roll;

            // 最终左右轮法向力
            N_L = N0 + deltaN_long - deltaN_total;
            N_R = N0 + deltaN_long + deltaN_total;
        }
        Eigen::MatrixXd parameters_bias_;          /**< Parameter bias used for computing cost for the test */
        std::vector<double> bias_thresholds_; /**< Threshold to determine whether two trajectories are equal */
        std::vector<double> std_dev_;         /**< Standard deviation used for generating noisy parameters */
        Eigen::MatrixXd smoothing_M_;         /**< Matrix used for smoothing the trajectory */
        VehicleParams param_;
        Eigen::Vector3f cur_pose_;
        Eigen::Quaterniond q_;
    };

    class STOMPSmoother {
    public:
        explicit STOMPSmoother();

        void init(ros::NodeHandle& nh);

        bool smooth(std::vector<Eigen::Vector3f>& path);

        void setOdom(const Eigen::Vector3d& odom_pose, const Eigen::Quaterniond& quaternion_input);

    private:
        std::shared_ptr<Stomp> stomp_ptr_;
        ros::NodeHandle node_;
        Eigen::Vector3f cur_pose_;
        Eigen::Quaterniond q_;
        ros::Publisher result_path_pub_;
    };
}


#endif //SRC_STOMP_SMOOTHER_HPP
