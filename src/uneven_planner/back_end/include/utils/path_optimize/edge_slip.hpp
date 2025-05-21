//
// Created by mazheng on 25-5-19.
//

#ifndef SRC_EDGE_SLIP_HPP
#define SRC_EDGE_SLIP_HPP

#include <iostream>
#include "edge_base.hpp"
#include "utils/math_util.hpp"
#include "penalty_bound.hpp"
#include "vertex_pose.hpp"
#include "utils/math_util.hpp"

using namespace uneven_planner;
namespace ninebot_algo::motion_planner {
    class EdgeSlipWork : public BaseBinaryEdge<1, double, VertexPose, VertexPose> {
    public:
        EdgeSlipWork() {
            _measurement = 0.0f;
        }

        void computeError() override {
            auto pose_1 = dynamic_cast<const VertexPose*>(_vertices[0]);
            auto pose_2 = dynamic_cast<const VertexPose*>(_vertices[1]);
            Eigen::Vector3d cur_pose = pose_1->estimate();
            Eigen::Vector3d next_pose = pose_2->estimate();
            auto delta_theta = uneven_planner::wrapToPi(next_pose[2] - cur_pose[2]);
            auto length = (next_pose - cur_pose).head(2).norm();
            double psi_i = cur_pose[2];
            double pitch, roll;
            computePointAttitude(theta_slope_, psi_s_, psi_i, pitch, roll);
            double N_l, N_r;
            computeForcesImproved(pitch, roll, N_l, N_r, param_);
            const double mu = 0.6f;
            auto F_l = mu * N_l;
            auto F_r = mu * N_r;
            Eigen::Vector3d pose_diff = next_pose - cur_pose;
            float connection_angle = atan2(pose_diff.y(), pose_diff.x());
            float rotate_sum = abs(uneven_planner::wrapToPi(connection_angle - cur_pose[2]))
                               + abs(uneven_planner::wrapToPi(next_pose[2] - connection_angle));
            auto forward = true;
            if (rotate_sum > M_PI) {
                forward = false;
            }
            auto dir = forward ? 1 : -1;
            auto d_r = dir * length + param_.wheelbase / 2 * delta_theta;
            auto d_l = dir * length - param_.wheelbase / 2 * delta_theta;
            if (length < 1e-6 && std::fabs(delta_theta) < 1e-6) {
                _error[0] = 0.0f;
                return;
            }
            auto w_drive = F_l * std::fabs(d_l) + F_r * std::fabs(d_r);
            auto a = tan(theta_slope_) * cos(psi_s_);
            auto b = tan(theta_slope_) * sin(psi_s_);
            auto delta_z = a * (next_pose.x() - cur_pose.x()) + b * (next_pose.y() - cur_pose.y())
                           + 0.09 * (a * (cos(next_pose.z()) - cos(cur_pose.z())) + b * (sin(next_pose.z()) - sin(cur_pose.z())));
            auto w_grav = param_.mass * param_.g * delta_z;
            auto delta_w = w_drive - w_grav;
            auto cost = 1 / delta_w * (std::fabs(d_r) + std::fabs(d_l)) / 2;
            std::cout << "debug path current pose: " << cur_pose[0] << " " << cur_pose[1] << " " << cur_pose[2] / M_PI * 180
                      << " next pose: " << next_pose[0] << " " << next_pose[1] << " " << next_pose[2] / M_PI * 180
                      << " pitch: " << pitch / M_PI * 180 << " roll: " << roll / M_PI * 180 << " and forces: " << N_l
                      << "," << N_r << " force: " << F_l << " " << F_r << " dist: " << d_l << " " << d_r
                      << " forward: " << forward << " work drive: " << w_drive << " delta z: " << delta_z
                      << " work grav: " << w_grav << " cost: " << cost << std::endl;
            _error[0] = cost;
        }

        void setSlopeInfo(const float& theta_slope, const float& psi_s, const VehicleParams& param) {
            theta_slope_ = theta_slope;
            psi_s_ = psi_s;
            param_ = param;
        }

    protected:
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

        float theta_slope_;
        float psi_s_;
        VehicleParams param_;
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    class EdgeSlip : public BaseMultiEdge<1, double> {
    public:
        EdgeSlip() {
            this->resize(5);
        }

        void computeError() override {
            auto pose_1 = dynamic_cast<const VertexPose*>(_vertices[0]);
            auto pose_2 = dynamic_cast<const VertexPose*>(_vertices[1]);
            auto pose_3 = dynamic_cast<const VertexPose*>(_vertices[2]);
            auto dt_1 = dynamic_cast<const VertexTimeDiff*>(_vertices[3]);
            auto dt_2 = dynamic_cast<const VertexTimeDiff*>(_vertices[4]);

            // VELOCITY & ACCELERATION
            const Eigen::Vector3d diff_1 = pose_2->estimate() - pose_1->estimate();
            const Eigen::Vector3d diff_2 = pose_3->estimate() - pose_2->estimate();

            double dist_1 = diff_1.head(2).norm();
            double dist_2 = diff_2.head(2).norm();
            const double angle_diff_1 = g2o::normalize_theta(diff_1[2]);
            const double angle_diff_2 = g2o::normalize_theta(diff_2[2]);

            double vel1 = dist_1 / dt_1->dt();
            double vel2 = dist_2 / dt_2->dt();

            // Diff dist projection to current pose direction
            vel1 *= fast_sigmoid(100
                                 * (diff_1[0] * cos(pose_1->estimate()[2])
                                    + diff_1[1] * sin(pose_1->estimate()[2])));
            vel2 *= fast_sigmoid(100
                                 * (diff_2[0] * cos(pose_2->estimate()[2])
                                    + diff_2[1] * sin(pose_2->estimate()[2])));

            double acc_v = (vel2 - vel1) * 2 / (dt_1->dt() + dt_2->dt());
            const double omega = angle_diff_1 / dt_1->dt();
            Eigen::Vector3d cur_pose = pose_1->estimate();
            Eigen::Vector3d next_pose = pose_2->estimate();
            auto delta_theta = uneven_planner::wrapToPi(next_pose[2] - cur_pose[2]);
            auto length = (next_pose - cur_pose).head(2).norm();
            double psi_i = cur_pose[2];
            double pitch, roll;
            computePointAttitude(theta_slope_, psi_s_, psi_i, pitch, roll);
            double N_l, N_r;
            computeForcesImproved(pitch, roll, N_l, N_r, param_);
            double n_l_next, n_r_next;
            psi_i = next_pose[2];
            computePointAttitude(theta_slope_, psi_s_, psi_i, pitch, roll);
            computeForcesImproved(pitch, roll, n_l_next, n_r_next, param_);
            N_l = (N_l + n_l_next) / 2.0f;
            N_r = (N_r + n_r_next) / 2.0f;
            if (length < 1e-6 && std::fabs(delta_theta) < 1e-6) {
                _error[0] = 0.0f;
                return;
            }
            auto v_l = vel1 - param_.wheelbase / 2 * omega;
            auto v_r = vel1 + param_.wheelbase / 2 * omega;
            auto f_total = param_.mass * std::fabs(acc_v + param_.g * sin(theta_slope_) * cos(psi_i - psi_s_));
            float ratio_r, ratio_l;
            if (std::fabs(v_l) < 1e-6 && std::fabs(v_r) < 1e-6) {
                ratio_r = 0.5f;
                ratio_l = 0.5f;
            } else {
                ratio_r = fabs(v_r) / (fabs(v_l) + fabs(v_r));
                ratio_l = fabs(v_l) / (fabs(v_l) + fabs(v_r));
            }
            auto f_l = f_total * ratio_l;
            auto f_r = f_total * ratio_r;
            auto mu_r = f_r / N_r;
            auto mu_l = f_l / N_l;
            std::cout << "debug path current pose: " << cur_pose[0] << " " << cur_pose[1] << " " << cur_pose[2] / M_PI * 180
                << " next pose: " << next_pose[0] << " " << next_pose[1] << " " << next_pose[2] / M_PI * 180 << " "
                << psi_i / M_PI * 180 << " pitch: " << pitch / M_PI * 180 << " roll: " << roll / M_PI * 180
                << " and forces: " << N_l << "," << N_r << " speed:" << vel1 << " " << omega << " acc: " << acc_v
                << " force total: " << f_total << " " << f_l << " " << f_r << " mu:" << mu_l << " " << mu_r << std::endl;
            auto mu_cost = std::max(mu_r, mu_l);
            _error[0] = penaltyBoundToInterval(mu_cost, cfg_->robot.max_mu, PENALTY_BOUND_EPSILON);
        }

        void setSlopeInfo(const float& theta_slope, const float& psi_s, const VehicleParams& param) {
            theta_slope_ = theta_slope;
            psi_s_ = psi_s;
            param_ = param;
        }

    protected:
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

        float theta_slope_;
        float psi_s_;
        VehicleParams param_;
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    class EdgeSlipStart : public BaseMultiEdge<1, double> {
    public:
        EdgeSlipStart() {
            this->resize(3);
        }

        void computeError() override {
            auto pose_1 = dynamic_cast<const VertexPose*>(_vertices[0]);
            auto pose_2 = dynamic_cast<const VertexPose*>(_vertices[1]);
            auto dt = dynamic_cast<const VertexTimeDiff*>(_vertices[2]);

            // VELOCITY & ACCELERATION
            const Eigen::Vector3d diff = pose_2->estimate() - pose_1->estimate();

            double dist = diff.head(2).norm();
            const double angle_diff = g2o::normalize_theta(diff[2]);

            double vel1 = 0.0f;
            double vel2 = dist / dt->dt();
            vel2 *= fast_sigmoid(100
                                 * (diff[0] * cos(pose_1->estimate()[2])
                                    + diff[1] * sin(pose_1->estimate()[2])));

            double acc_v = (vel2 - vel1) / dt->dt();
            const double omega = 0.0f;
            Eigen::Vector3d cur_pose = pose_1->estimate();
            Eigen::Vector3d next_pose = pose_2->estimate();
            auto delta_theta = uneven_planner::wrapToPi(next_pose[2] - cur_pose[2]);
            auto length = (next_pose - cur_pose).head(2).norm();
            double psi_i = cur_pose[2];
            double pitch, roll;
            computePointAttitude(theta_slope_, psi_s_, psi_i, pitch, roll);
            double N_l, N_r;
            computeForcesImproved(pitch, roll, N_l, N_r, param_);
            double n_l_next, n_r_next;
            psi_i = next_pose[2];
            computePointAttitude(theta_slope_, psi_s_, psi_i, pitch, roll);
            computeForcesImproved(pitch, roll, n_l_next, n_r_next, param_);
            N_l = (N_l + n_l_next) / 2.0f;
            N_r = (N_r + n_r_next) / 2.0f;
            if (length < 1e-6 && std::fabs(delta_theta) < 1e-6) {
                _error[0] = 0.0f;
                return;
            }
            auto v_l = vel1 - param_.wheelbase / 2 * omega;
            auto v_r = vel1 + param_.wheelbase / 2 * omega;
            auto f_total = param_.mass * param_.rearWeightFraction * std::fabs(acc_v + param_.g * sin(theta_slope_) * cos(psi_i - psi_s_));
            float ratio_r, ratio_l;
            if (std::fabs(v_l) < 1e-6 && std::fabs(v_r) < 1e-6) {
                ratio_r = 0.5f;
                ratio_l = 0.5f;
            } else {
                ratio_r = fabs(v_r) / (fabs(v_l) + fabs(v_r));
                ratio_l = fabs(v_l) / (fabs(v_l) + fabs(v_r));
            }

            auto f_l = f_total * ratio_l;
            auto f_r = f_total * ratio_r;
            auto mu_r = f_r / N_r;
            auto mu_l = f_l / N_l;
//            auto w_drive = F_l * std::fabs(d_l) + F_r * std::fabs(d_r);
//            auto a = tan(theta_slope_) * cos(psi_s_);
//            auto b = tan(theta_slope_) * sin(psi_s_);
//            auto delta_z = a * (next_pose.x() - cur_pose.x()) + b * (next_pose.y() - cur_pose.y())
//                           + 0.09 * (a * (cos(next_pose.z()) - cos(cur_pose.z())) + b * (sin(next_pose.z()) - sin(cur_pose.z())));
//            auto w_grav = param_.mass * param_.g * delta_z;
//            auto delta_w = w_drive - w_grav;
            auto mu_cost = std::max(mu_r, mu_l);
            _error[0] = penaltyBoundToInterval(mu_cost, cfg_->robot.max_mu, PENALTY_BOUND_EPSILON);
        }

        void setSlopeInfo(const float& theta_slope, const float& psi_s, const VehicleParams& param) {
            theta_slope_ = theta_slope;
            psi_s_ = psi_s;
            param_ = param;
        }

    protected:
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

        float theta_slope_;
        float psi_s_;
        VehicleParams param_;
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    class EdgeSlipGoal : public BaseMultiEdge<1, double> {
    public:
        EdgeSlipGoal() {
            this->resize(3);
        }

        void computeError() override {
            auto pose_1 = dynamic_cast<const VertexPose*>(_vertices[0]);
            auto pose_2 = dynamic_cast<const VertexPose*>(_vertices[1]);
            auto dt = dynamic_cast<const VertexTimeDiff*>(_vertices[2]);

            // VELOCITY & ACCELERATION
            const Eigen::Vector3d diff_1 = pose_2->estimate() - pose_1->estimate();

            double dist_1 = diff_1.head(2).norm();
            const double angle_diff_1 = g2o::normalize_theta(diff_1[2]);

            // Use arc for calculation
            //            if (angle_diff_1 != 0) {
            //                dist_1 = fabs(angle_diff_1 * (dist_1 / (2 * sin(angle_diff_1 / 2))));
            //            }
            //            if (angle_diff_2 != 0) {
            //                dist_2 = fabs(angle_diff_2 * (dist_2 / (2 * sin(angle_diff_2 / 2))));
            //            }

            double vel1 = dist_1 / dt->dt();
            double vel2 = 0.0f;

            //                vel1 *= g2o::sign(diff_1[0] * cos(pose_1->estimate()[2]) + diff_1[1] * sin(pose_1->estimate()[2]));
            //                vel2 *= g2o::sign(diff_2[0] * cos(pose_2->estimate()[2]) + diff_2[1] * sin(pose_2->estimate()[2]));
            // Diff dist projection to current pose direction
            vel1 *= fast_sigmoid(100
                                 * (diff_1[0] * cos(pose_1->estimate()[2])
                                    + diff_1[1] * sin(pose_1->estimate()[2])));
            vel2 = 0.0f;

            double acc_v = (vel2 - vel1) / (dt->dt());
            const double omega = angle_diff_1 / dt->dt();
            Eigen::Vector3d cur_pose = pose_1->estimate();
            Eigen::Vector3d next_pose = pose_2->estimate();
            auto delta_theta = uneven_planner::wrapToPi(next_pose[2] - cur_pose[2]);
            auto length = (next_pose - cur_pose).head(2).norm();
            double psi_i = cur_pose[2];
            double pitch, roll;
            computePointAttitude(theta_slope_, psi_s_, psi_i, pitch, roll);
            double N_l, N_r;
            computeForcesImproved(pitch, roll, N_l, N_r, param_);
            double n_l_next, n_r_next;
            psi_i = next_pose[2];
            computePointAttitude(theta_slope_, psi_s_, psi_i, pitch, roll);
            computeForcesImproved(pitch, roll, n_l_next, n_r_next, param_);
            N_l = (N_l + n_l_next) / 2.0f;
            N_r = (N_r + n_r_next) / 2.0f;
            if (length < 1e-6 && std::fabs(delta_theta) < 1e-6) {
                _error[0] = 0.0f;
                return;
            }
            auto v_l = vel1 - param_.wheelbase / 2 * omega;
            auto v_r = vel1 + param_.wheelbase / 2 * omega;
            auto f_total = param_.mass * param_.rearWeightFraction * std::fabs(acc_v + param_.g * sin(theta_slope_) * cos(psi_i - psi_s_));
            float ratio_r, ratio_l;
            if (std::fabs(v_l) < 1e-6 && std::fabs(v_r) < 1e-6) {
                ratio_r = 0.5f;
                ratio_l = 0.5f;
            } else {
                ratio_r = fabs(v_r) / (fabs(v_l) + fabs(v_r));
                ratio_l = fabs(v_l) / (fabs(v_l) + fabs(v_r));
            }
            auto f_l = f_total * ratio_l;
            auto f_r = f_total * ratio_r;
            auto mu_r = f_r / N_r;
            auto mu_l = f_l / N_l;
//            auto w_drive = F_l * std::fabs(d_l) + F_r * std::fabs(d_r);
//            auto a = tan(theta_slope_) * cos(psi_s_);
//            auto b = tan(theta_slope_) * sin(psi_s_);
//            auto delta_z = a * (next_pose.x() - cur_pose.x()) + b * (next_pose.y() - cur_pose.y())
//                           + 0.09 * (a * (cos(next_pose.z()) - cos(cur_pose.z())) + b * (sin(next_pose.z()) - sin(cur_pose.z())));
//            auto w_grav = param_.mass * param_.g * delta_z;
//            auto delta_w = w_drive - w_grav;
            auto cost = pow(mu_r, 2) + pow(mu_l, 2);
            auto mu_cost = std::max(mu_r, mu_l);
            _error[0] = penaltyBoundToInterval(mu_cost, cfg_->robot.max_mu, PENALTY_BOUND_EPSILON);
        }

        void setSlopeInfo(const float& theta_slope, const float& psi_s, const VehicleParams& param) {
            theta_slope_ = theta_slope;
            psi_s_ = psi_s;
            param_ = param;
        }

    protected:
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

        float theta_slope_;
        float psi_s_;
        VehicleParams param_;
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
}


#endif //SRC_EDGE_SLIP_HPP
