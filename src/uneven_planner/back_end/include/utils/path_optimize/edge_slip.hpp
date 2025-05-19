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
    class EdgeSlip : public BaseBinaryEdge<1, double, VertexPose, VertexPose> {
    public:
        EdgeSlip() {
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
            const double mu = 1.2f;
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
            auto cost = 1 / delta_w;
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
}


#endif //SRC_EDGE_SLIP_HPP
