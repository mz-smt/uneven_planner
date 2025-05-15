//
// Created by mazheng on 25-4-28.
//

#ifndef SRC_PSO_SMOOTHER_HPP
#define SRC_PSO_SMOOTHER_HPP

#include <Eigen/Core>
#include <utils/eta3_spline.hpp>
#include <memory>
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Geometry>
#include "utils/math_util.hpp"

namespace uneven_planner {

    struct SmoothTrackerParam {
        SmoothTrackerParam()
                : max_vel_x(0.4f),
                  max_vel_x_backward(0.2f),
                  max_vel_theta(0.5f),
                  bound_x(-0.5f, 0.5f),
                  bound_y(-0.5f, 0.5f),
                  bound_theta(-M_PI, M_PI),
                  max_backward_dis(0.3f),
                  weight_penalty_backward(500.0f),
                  weight_penalty_gear_switch(0.5f),
                  weight_penalty_work(20.0f),
                  verbose(true) {}
        float max_vel_x;
        float max_vel_x_backward;
        float max_vel_theta;
        Eigen::Vector2f bound_x;  // lower,upper
        Eigen::Vector2f bound_y;
        Eigen::Vector2f bound_theta;

        float max_backward_dis;
        float weight_penalty_backward;
        float weight_penalty_gear_switch;
        float weight_penalty_work;

        bool verbose;
    };

    class PathCostCalculator {
    public:
        PathCostCalculator() = default;

        static void setParam(const SmoothTrackerParam& param) {
            tracker_param_ = param;
            //            ALOGD("Param max vel %.2f, min vel %.2f, max angular vel %.2f.",
            //                  max_forward_linear_vel_, max_backward_linear_vel_, max_angular_vel_);
        }

        /**
         * @param start_point_body start point must in body frame
         * @param end_point_body end point must in body frame
         * */
        static void setBoundaryCondition(const Eigen::Vector3f& start_point_body,
                                         const Eigen::Vector3f& end_point_body,
                                         const Eigen::Vector3f& cur_pose,
                                         const Eigen::Quaterniond& q) {
            start_point_ = start_point_body;
            end_point_ = end_point_body;
            cur_pose_ = cur_pose;
            q_ = q;
            //            ALOGD("SetBoundaryCondition start(%.2f,%.2f,%.2f), end(%.2f,%.2f,%.2f).",
            //                  start_point_[0], start_point_[1], start_point_[2],
            //                  end_point_[0], end_point_[1], end_point_[2]);
        }
        static void setVehicleParams(const VehicleParams& vehicle_param) {
            vehicle_param_ = vehicle_param;
        }
        /**
         * @brief calculate spline time cost with collision penalty and backward distance penalty.
         * @param xa,ya,theta_a  start point
         * @param xb,yb,theta_b  end point
         * @param is_reverse True:spline backward move. False: spline forward move.
         * */
        static double calculateSplineTime(float xa, float ya, float theta_a, float xb, float yb,
                                          float theta_b, bool is_reverse) {
            double theta_slope, psi_s;
            computeSlopeAngles(q_, vehicle_param_.g, cur_pose_[2], theta_slope, psi_s);
            double spline_cost = 0;
            auto max_linear_vel = is_reverse ? std::fabs(tracker_param_.max_vel_x_backward)
                                             : std::fabs(tracker_param_.max_vel_x);
            auto max_angular_vel = std::fabs(tracker_param_.max_vel_theta);
            if (is_reverse) {
                theta_a += M_PI;
                theta_b += M_PI;
            }
            auto spline =
                    std::make_unique<Eta3Spline>(xa, ya, theta_a, 0, 0, xb, yb, theta_b, 0, 0);
            auto point_dist = hypot(yb - ya, xb - xa);
            auto max_check_num = (int)std::fmax(5.0f, point_dist / step_size_ + 1);

            Eigen::Vector3f last_position;
            for (int j = 0; j <= max_check_num; ++j) {
                auto pos = spline->evaluate3D((float)j / (float)max_check_num);
                if (is_reverse) pos[2] = (float)wrapToPi(pos[2] + M_PI);

                /// calculate time
                if (j != 0) {
                    auto dist_cost = (pos.head(2) - last_position.head(2)).norm() / max_linear_vel;
                    auto angle_cost =
                            std::fabs(wrapToPi(pos(2) - last_position(2))) / max_angular_vel;
                    spline_cost += std::fmax(dist_cost, angle_cost);
                    /// force penalty
                    double pitch, roll;
                    auto cur_pose = last_position;
                    auto next_pose = pos;
                    auto delta_theta = wrapToPi(next_pose[2] - cur_pose[2]);
                    auto length = (next_pose - cur_pose).head(2).norm();
                    double psi_i = cur_pose[2];
                    computePointAttitude(theta_slope, psi_s, psi_i, pitch, roll);
                    double N_l, N_r;
                    computeForcesImproved(pitch, roll, N_l, N_r, vehicle_param_);
                    const double mu = 1.0f;
                    auto F_l = mu * N_l;
                    auto F_r = mu * N_r;
                    auto dir = is_reverse ? -1 : 1;
                    auto d_r = dir * length + vehicle_param_.wheelbase / 2 * delta_theta;
                    auto d_l = dir * length - vehicle_param_.wheelbase / 2 * delta_theta;
                    if (length < 1e-6 && std::fabs(delta_theta) < 1e-6) {
                        continue;
                    }
                    auto w_drive = F_l * std::fabs(d_l) + F_r * std::fabs(d_r);
                    auto a = tan(theta_slope) * cos(psi_s);
                    auto b = tan(theta_slope) * sin(psi_s);
                    auto delta_z = a * (next_pose.x() - cur_pose.x()) + b * (next_pose.y() - cur_pose.y())
                                   + 0.09 * (a * (cos(next_pose.z()) - cos(cur_pose.z())) + b * (sin(next_pose.z()) - sin(cur_pose.z())));
                    auto w_grav = vehicle_param_.mass * vehicle_param_.g * delta_z;
                    auto delta_w = w_drive;
                    auto cost = 1 / delta_w;
                    std::cout << "debug current: " << cur_pose[0] << " " << cur_pose[1] << " " << cur_pose[2] / M_PI * 180
                              << " to next point: " << next_pose[0] << " " << next_pose[1] << " " << next_pose[2] / M_PI * 180
                              << " path dist: " << length << " theta diff: " << delta_theta << " pitch: "
                              << pitch / M_PI * 180 << " roll: " << roll / M_PI * 180 << " and forces: " << N_l << "," << N_r
                              << " force: " << F_l << " " << F_r << " dist: " << d_l << " " << d_r << " work drive: " << w_drive
                              << " delta z: " << delta_z << " work grav: " << w_grav << " cost: " << cost << std::endl;
                    spline_cost += cost * tracker_param_.weight_penalty_work;
                }
                last_position = std::move(pos);
            }

            /// penalty backward distance
            if (is_reverse && point_dist > tracker_param_.max_backward_dis)
                spline_cost += tracker_param_.weight_penalty_backward;
            /// additional penalty
            return spline_cost;
        }

        // TODO: Optimize: use precise time considering acc and dcc
        /**
         * @brief Calculate trajectory time cost
         * @param path_points Trajectory point(x,y,theta) in a col
         * @return Time cost
         * */
        template<typename Derived>
        double operator()(const Eigen::MatrixBase<Derived>& path_points) const {
            // point(x,y,theta)
            int points_size = (int)path_points.size() / 3;
            assert(points_size >= 1);
            assert(path_points.size() % 3 == 0);

            float xa, ya, theta_a, xb, yb, theta_b;
            double spline_cost = 0;
            int8_t gear_switch = 0;
            for (int i = 0; i <= points_size; ++i) {
                // point a
                if (i == 0) {
                    xa = start_point_[0];
                    ya = start_point_[1];
                    theta_a = start_point_[2];
                } else {
                    xa = xb;
                    ya = yb;
                    theta_a = theta_b;
                }

                // point b
                if (i == points_size) {
                    xb = end_point_[0];
                    yb = end_point_[1];
                    theta_b = end_point_[2];
                } else {
                    auto index = 3 * i;
                    xb = path_points(index);
                    yb = path_points(++index);
                    theta_b = path_points(++index);
                }
                // min(spline_forward_value, spline_back_value)
                auto spline_forward_cost =
                        calculateSplineTime(xa, ya, theta_a, xb, yb, theta_b, false);
                auto spline_back_cost = calculateSplineTime(xa, ya, theta_a, xb, yb, theta_b, true);
                //                spline_cost += std::min(spline_forward_cost, spline_back_cost);
                auto&& cost_forward =
                        (gear_switch < 0
                         ? (spline_forward_cost + tracker_param_.weight_penalty_gear_switch)
                         : spline_forward_cost);
                auto&& cost_back =
                        (gear_switch > 0
                         ? (spline_back_cost + tracker_param_.weight_penalty_gear_switch)
                         : spline_back_cost);
                gear_switch = cost_forward <= cost_back ? 1 : -1;
                spline_cost += gear_switch > 0 ? cost_forward : cost_back;
            }
            //            ALOGD("total_spline_cost: %.2f.", spline_cost);
            return spline_cost;
        }

    private:
        static Eigen::Vector3f start_point_;
        static Eigen::Vector3f end_point_;
        static Eigen::Vector3f cur_pose_;
        static Eigen::Quaterniond q_;
        static VehicleParams vehicle_param_;

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
    public:
        static SmoothTrackerParam tracker_param_;
        static float step_size_;
    };

    class PSOSmoother {
    public:
        explicit PSOSmoother();

        void init(ros::NodeHandle& nh);

        bool smooth(std::vector<Eigen::Vector3f>& path);

        void setOdom(const Eigen::Vector3d& odom_pose, const Eigen::Quaterniond& quaternion_input);

    private:
        void refactorPath(const std::vector<Eigen::Vector3f>& init_path, std::vector<Eigen::Vector3f>& refactor_path,
                          int& points_num);

        ros::NodeHandle node_;

        SmoothTrackerParam param_;

        std::vector<Eigen::Vector3f> m_init_path_;
        std::vector<Eigen::Vector3f> m_optimal_path_;
        std::vector<Eigen::Vector3f> result_path_;

        const float StepSize = 0.5f;
        const int MinPointSize = 3;
        const int MaxPointSize = 5;
        const float PathStep = 0.1f;
        const int MaxIterNum = 20;
        const int MaxTheadNum = 4;
        Eigen::Vector3f cur_pose_;
        Eigen::Quaterniond q_;

        Eigen::Vector3f speed_limit_;

        ros::Publisher refactor_path_pub_;
        ros::Publisher result_path_pub_;
    };
}



#endif //SRC_PSO_SMOOTHER_HPP
