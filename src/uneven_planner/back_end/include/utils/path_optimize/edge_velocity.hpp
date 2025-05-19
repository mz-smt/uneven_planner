//
// Created by evanlin on 5/27/20.
//

#ifndef MOTIONPLANNER_EDGEVELOCITY_HPP
#define MOTIONPLANNER_EDGEVELOCITY_HPP

#include "edge_base.hpp"
#include "penalty_bound.hpp"
#include "vertex_pose.hpp"
#include "vertex_time_diff.hpp"

namespace ninebot_algo {
    namespace motion_planner {

        // todo: Add EdgeVelocityStar for gear direction penalty
        class EdgeVelocity : public BaseMultiEdge<2, double> {
        public:
            EdgeVelocity() {
                this->resize(3);
            }

            void computeError() override {
                auto pose_1 = dynamic_cast<const VertexPose*>(_vertices[0]);
                auto pose_2 = dynamic_cast<const VertexPose*>(_vertices[1]);
                auto delta_t = dynamic_cast<const VertexTimeDiff*>(_vertices[2]);

                const Eigen::Vector3d pose_diff = pose_2->estimate() - pose_1->estimate();
                double dist = pose_diff.head(2).norm();
                double angle_diff = g2o::normalize_theta(pose_diff[2]);

                /* Vel */
                // Calc arc length
                //            if (angle_diff != 0.0f) {
                //                dist = fabs(angle_diff * (dist / (2 * sin(angle_diff / 2))));
                //            }
                double vel = dist / delta_t->dt();
                vel *= fast_sigmoid(100
                                    * (pose_diff[0] * cos(pose_1->estimate()[2])
                                       + pose_diff[1] * sin(pose_1->estimate()[2])));

                /* Omega */
                const double omega = angle_diff / delta_t->dt();

                _error[0] =
                    penaltyBoundToInterval(vel, -cfg_->robot.max_lin_vel_backward,
                                           cfg_->robot.max_lin_vel_forward, PENALTY_BOUND_EPSILON);
                _error[1] =
                    penaltyBoundToInterval(omega, cfg_->robot.max_rot_vel, PENALTY_BOUND_EPSILON);

//                std::cout << "EdgeVelocity pose1:" << *pose_1
//                          << " pose2:" << *pose_2
//                          << " dt:" << delta_t->dt()
//                          << " dist:" << dist
//                          << " error0:" << _error[0]
//                          << " error1:" << _error[1] << std::endl;
#ifdef DEBUG_EDGE_ERROR
                if (_error[0] > 1e-1 || _error[1] > 1e-1) {
                    std::cout << "EdgeVelocity"
                              << " error0:" << _error[0] << " error1:" << _error[1] << std::endl;
                }
#endif  //DEBUG_EDGE_ERROR
            }

        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        };

    }  // namespace motion_planner
}  // namespace ninebot_algo

#endif  //MOTIONPLANNER_EDGEVELOCITY_HPP
