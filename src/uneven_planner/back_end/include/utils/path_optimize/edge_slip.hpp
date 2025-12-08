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
            if (length < 1e-6 && std::fabs(delta_theta) < 1e-6) {
                _error[0] = 0.0f;
                return;
            }
            double psi_i = cur_pose[2];
            double pitch, roll;
            computePointAttitude(theta_slope_, psi_s_, psi_i, pitch, roll);
            double N_l, N_r;
            computeForcesImproved(pitch, roll, N_l, N_r, param_);
            float connection_angle = atan2(next_pose.y() - cur_pose.y(), next_pose.x() - cur_pose.x());
            float rotate_sum = abs(wrapToPi(connection_angle - cur_pose[2]))
                               + abs(wrapToPi(next_pose[2] - connection_angle));
            auto forward = true;
            if (rotate_sum > M_PI) {
                forward = false;
            }
            auto weight_dir = forward ? 1.0f : -1.0f;
            auto turn_left = delta_theta > 0;
            auto weight_turn = turn_left ? 1.0f : -1.0f;
            auto delta_slope_heading = wrapToPi(psi_i - psi_s_);
            const double mu = 0.8f;
            auto F_l = mu * N_l;
            auto F_r = mu * N_r;
            float dist_r, dist_l;
            float radius_r, radius_l;
            float R = 0.0f;
            if (std::fabs(delta_theta) < 1e-6) {
                dist_r = length;
                dist_l = length;
                R = 100;
                radius_l = R;
                radius_r = R;
            } else {
                R = std::fabs(length / (2 * sin(delta_theta / 2.0f)));
                if (forward == turn_left) {
                    radius_r = (R + param_.wheelbase / 2);
                    radius_l = (R - param_.wheelbase / 2);
                } else {
                    radius_r = (R - param_.wheelbase / 2);
                    radius_l = (R + param_.wheelbase / 2);
                }
                dist_r = radius_r * std::fabs(delta_theta);
                dist_l = radius_l * std::fabs(delta_theta);
            }
            auto d = 0.1;
            auto t_g_d = R * cos(delta_slope_heading) - d * sin(delta_slope_heading) * weight_turn;
            auto T_g = weight_dir * param_.mass * param_.g * sin(theta_slope_) * t_g_d;
            auto wheel_torque = F_r * radius_r + F_l * radius_l;
            auto torque = std::fabs(wheel_torque) - T_g;
            if (torque < 0) {
                torque = 0.1;
            }
            auto cost_torque = 1.0 / torque;
            auto w_drive = F_l * std::fabs(dist_l) + F_r * std::fabs(dist_r);
            auto a = tan(theta_slope_) * cos(psi_s_);
            auto b = tan(theta_slope_) * sin(psi_s_);
            auto delta_z = a * (next_pose.x() - cur_pose.x()) + b * (next_pose.y() - cur_pose.y())
                           + 0.0 * (a * (cos(next_pose.z()) - cos(cur_pose.z())) + b * (sin(next_pose.z()) - sin(cur_pose.z())));
            auto w_grav = param_.mass * param_.g * delta_z;
            auto delta_w = w_drive - w_grav;
            if (delta_w < 0) {
                delta_w = 1e-6;
            }
            auto cost_work = 1 / delta_w * (std::fabs(dist_r) + std::fabs(dist_l)) / 2;
            if (pitch > 0 && delta_z > 0 && !forward) {
                auto penalty_work_cost = cost_work * cos(delta_slope_heading + M_PI) * 50;
                cost_work += penalty_work_cost;
            }
            float cost_roll = 0.0f;
            if (roll > 0) {
                if (forward != turn_left) {
                    cost_roll = std::fabs(F_r / F_l - dist_r / dist_l);
                }
            } else {
                if (forward == turn_left) {
                    cost_roll = std::fabs(F_l / F_r - dist_l / dist_r);
                }
            }
            cost_roll *= std::fabs(sin(delta_slope_heading));
            auto cost = cost_work * 5.0 + cost_roll * 0.5 + cost_torque * 5.0;
            std::cout << "debug path current pose: " << cur_pose[0] << " " << cur_pose[1] << " " << cur_pose[2] / M_PI * 180
                      << " next pose: " << next_pose[0] << " " << next_pose[1] << " " << next_pose[2] / M_PI * 180 << " "
                      << psi_i / M_PI * 180 << " pitch: " << pitch / M_PI * 180 << " roll: " << roll / M_PI * 180
                      << " and N-forces: " << N_l << "," << N_r << " force: " << F_l << "," << F_r << " dist: "
                      << dist_l << "," << dist_r << " work: " << w_drive << " , " << w_grav
                      << " direction: " << forward << " " << turn_left << " R:" << R << " cost roll: " << cost_roll
                      << " cost work: " << cost_work << " cost torque:" << cost_torque << " cost: " << cost << std::endl;
            _error[0] = cost;
        }

        void setSlopeInfo(const float& theta_slope, const float& psi_s, const VehicleParams& param) {
            theta_slope_ = theta_slope;
            psi_s_ = psi_s;
            param_ = param;
        }

        void linearizeOplus() override {
            VertexXiType* vi = static_cast<VertexXiType*>(_vertices[0]);
            VertexXjType* vj = static_cast<VertexXjType*>(_vertices[1]);

            bool iNotFixed = !(vi->fixed());
            bool jNotFixed = !(vj->fixed());

            if (!iNotFixed && !jNotFixed)
                return;

            const number_t delta = g2o::cst(1e-2);
            const number_t scalar = 1 / (2*delta);
            ErrorVector errorBak;
            ErrorVector errorBeforeNumeric = _error;

            // A statically allocated array is far and away the most efficient
            // way to construct the perturbation vector for the Jacobian. If the
            // dimension is known at compile time, use directly. If the
            // dimension is known at run time and is less than 12, use an
            // allocated array of up to 12. Otherwise, use a fallback of a
            // dynamically allocated array. The value of 12 is used because
            // most vertices have a dimension significantly smaller than this.

            if (iNotFixed) {
                g2o::internal::QuadraticFormLock lck(*vi);
                //Xi - estimate the jacobian numerically

                const int vi_dim = 3;

                if ((VertexXiType::Dimension >= 0) || (vi_dim <= 12)) {
                    number_t add_vi[(VertexXiType::Dimension >= 0) ? VertexXiType::Dimension : 12] = {};

                    // add small step along the unit vector in each dimension
                    for (int d = 0; d < vi_dim; ++d) {
                        vi->push();
                        add_vi[d] = delta;
                        vi->oplus(add_vi);
                        computeError();
                        errorBak = _error;
                        vi->pop();
                        vi->push();
                        add_vi[d] = -delta;
                        vi->oplus(add_vi);
                        computeError();
                        errorBak -= _error;
                        vi->pop();
                        add_vi[d] = 0.0;
                        _jacobianOplusXi.col(d) = scalar * errorBak;
                    } // end dimension
                }
                else {
                    g2o::dynamic_aligned_buffer<number_t> buffer{ size_t(vi_dim) };
                    number_t* add_vi = buffer.request(vi_dim);
                    std::fill(add_vi, add_vi + vi_dim, g2o::cst(0.0));

                    // add small step along the unit vector in each dimension
                    for (int d = 0; d < vi_dim; ++d) {
                        vi->push();
                        add_vi[d] = delta;
                        vi->oplus(add_vi);
                        computeError();
                        errorBak = _error;
                        vi->pop();
                        vi->push();
                        add_vi[d] = - delta;
                        vi->oplus(add_vi);
                        computeError();
                        errorBak -= _error;
                        vi->pop();
                        add_vi[d] = 0;
                        _jacobianOplusXi.col(d) = scalar * errorBak;
                    } // end dimension
                }
            }

            if (jNotFixed) {
                g2o::internal::QuadraticFormLock lck(*vj);
                //Xj - estimate the jacobian numerically

                const int vj_dim = 3;

                if ((VertexXjType::Dimension >= 0) || (vj_dim <= 12)) {
                    number_t add_vj[(VertexXjType::Dimension >= 0) ? VertexXjType::Dimension : 12] = {};

                    // add small step along the unit vector in each dimension
                    for (int d = 0; d < vj_dim; ++d) {
                        vj->push();
                        add_vj[d] = delta;
                        vj->oplus(add_vj);
                        computeError();
                        errorBak = _error;
                        vj->pop();
                        vj->push();
                        add_vj[d] = -delta;
                        vj->oplus(add_vj);
                        computeError();
                        errorBak -= _error;
                        vj->pop();
                        add_vj[d] = 0.0;

                        _jacobianOplusXj.col(d) = scalar * errorBak;
                    } // end dimension
                }
                else {
                    const int vj_dim = vj->dimension();
                    g2o::dynamic_aligned_buffer<number_t> buffer{ size_t(vj_dim) };
                    number_t* add_vj = buffer.request(vj_dim);
                    std::fill(add_vj, add_vj + vj_dim, g2o::cst(0.0));

                    // add small step along the unit vector in each dimension
                    for (int d = 0; d < vj_dim; ++d) {
                        vj->push();
                        add_vj[d] = delta;
                        vj->oplus(add_vj);
                        computeError();
                        errorBak = _error;
                        vj->pop();
                        vj->push();
                        add_vj[d] = - delta;
                        vj->oplus(add_vj);
                        computeError();
                        errorBak -= _error;
                        vj->pop();
                        add_vj[d] = 0;
                        _jacobianOplusXj.col(d) = scalar * errorBak;
                    } // end dimension
                }
            }
            _error = errorBeforeNumeric;
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
            // 1. 重力分量计算（车辆坐标系）
            double g_x = params.g * sin(pitch);                     // 纵向分量
            double g_y = params.g * sin(roll) * cos(pitch);         // 横向分量
            double g_z = params.g * cos(pitch) * cos(roll);         // 垂直分量

            // 2. 静态后轴载荷（考虑坡度）
            double N0 = (params.mass * g_z * params.rearWeightFraction) / 2.0;

            // 3. 载荷转移计算
            // 纵向载荷转移（使用轴距，上坡时后轴载荷增加）
            double deltaN_long = -(params.mass * g_x * params.cgHeight) / params.wheelbase / 2.0;  // ✅

            // 横向载荷转移（使用轮距）
            double deltaN_roll = (params.mass * g_y * params.cgHeight) / params.trackWidth;       // ✅

            // 4. 左右轮最终载荷
            N_L = N0 + deltaN_long - deltaN_roll;  // 左侧轮
            N_R = N0 + deltaN_long + deltaN_roll;  // 右侧轮
        }

        float theta_slope_;
        float psi_s_;
        VehicleParams param_;
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
}


#endif //SRC_EDGE_SLIP_HPP
