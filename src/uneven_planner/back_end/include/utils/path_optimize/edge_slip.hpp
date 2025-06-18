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
            float R = 0.0f;
            if (std::fabs(delta_theta) < 1e-6) {
                R = 100.0f;
            } else {
                R = std::fabs(length / (2 * sin(delta_theta / 2.0f)));
            }
            R = std::min(R, 100.0f);
            const double mu = 0.8f;
            auto F_l = mu * N_l;
            auto F_r = mu * N_r;
            float d_r, d_l;
            if (forward) {
                d_r = length + param_.wheelbase / 2 * delta_theta;
                d_l = length - param_.wheelbase / 2 * delta_theta;
            } else {
                d_r = length - param_.wheelbase / 2 * delta_theta;
                d_l = length + param_.wheelbase / 2 * delta_theta;
            }
            auto w_drive = F_l * std::fabs(d_l) + F_r * std::fabs(d_r);
            auto dx = next_pose[0] - cur_pose[0];
            auto dy = next_pose[1] - cur_pose[1];
            auto d_xy = dx * cos(psi_s_) + dy * sin(psi_s_);
            auto f_slope = param_.mass * param_.g * sin(theta_slope_);
            auto dist_slope = d_xy / cos(theta_slope_);
            auto w_grav = f_slope * dist_slope;
            auto delta_w = w_drive - w_grav;
            if (delta_w < 0) {
                delta_w = 1e-6;
            }
            auto cost_t = 1 / delta_w * (std::fabs(d_r) + std::fabs(d_l)) / 2;
            float cost_roll = 0.0f;
            if (roll > 0) {
                if (forward != turn_left) {
                    cost_roll = std::fabs(F_r / F_l - d_r / d_l) * 0.5;
                }
            } else {
                if (forward == turn_left) {
                    cost_roll = std::fabs(F_l / F_r - d_l / d_r) * 0.5;
                }
            }
            auto cost = cost_t * 10 + cost_roll;
            std::cout << "debug path current pose: " << cur_pose[0] << " " << cur_pose[1] << " " << cur_pose[2] / M_PI * 180
                      << " next pose: " << next_pose[0] << " " << next_pose[1] << " " << next_pose[2] / M_PI * 180 << " "
                      << psi_i / M_PI * 180 << " pitch: " << pitch / M_PI * 180 << " roll: " << roll / M_PI * 180
                      << " and N-forces: " << N_l << "," << N_r << " force: " << F_l << "," << F_r << " dist: "
                      << d_l << "," << d_r
                      << " direction: " << forward << " " << turn_left << " R:" << R << " cost roll: " << cost_roll
                      << " cost t: " << cost_t << " cost: " << cost << std::endl;
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
            // 重力分解（车辆姿态）：
            double g_x = params.g * sin(pitch);  // 纵向分量
            double g_y = params.g * sin(roll);  // 横向分量
            double g_z = params.g * cos(pitch) * cos(roll);  // 垂直分量

            // 静态后轴法向力（后轮）：
            double N0 = (params.mass * g_z * params.rearWeightFraction) / 2.0;

            // 载荷转移：
            // 纵向载荷转移（由俯仰角引起）
            double deltaN_long =
                    - (params.mass * params.g * sin(pitch) * cos(roll) * params.cgHeight) / params.trackWidth / 2.0f;
            // 横向载荷转移（由横滚角引起）
            double deltaN_roll =
                    (params.mass * params.g * sin(roll) * cos(pitch) * params.cgHeight) / params.wheelbase;
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
