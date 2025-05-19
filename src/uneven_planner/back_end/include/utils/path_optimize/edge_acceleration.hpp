//
// Created by evanlin on 5/27/20.
//

#ifndef MOTIONPLANNER_EDGEACCELERATION_HPP
#define MOTIONPLANNER_EDGEACCELERATION_HPP

#include "edge_base.hpp"
#include "optimize_params.hpp"
#include "penalty_bound.hpp"
#include "vertex_pose.hpp"
#include "vertex_time_diff.hpp"

namespace ninebot_algo {
    namespace motion_planner {

        class EdgeAcceleration : public BaseMultiEdge<2, double> {
        public:
            EdgeAcceleration() {
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

                // Use arc for calculation
                //            if (angle_diff_1 != 0) {
                //                dist_1 = fabs(angle_diff_1 * (dist_1 / (2 * sin(angle_diff_1 / 2))));
                //            }
                //            if (angle_diff_2 != 0) {
                //                dist_2 = fabs(angle_diff_2 * (dist_2 / (2 * sin(angle_diff_2 / 2))));
                //            }

                double vel1 = dist_1 / dt_1->dt();
                double vel2 = dist_2 / dt_2->dt();

                //                vel1 *= g2o::sign(diff_1[0] * cos(pose_1->estimate()[2]) + diff_1[1] * sin(pose_1->estimate()[2]));
                //                vel2 *= g2o::sign(diff_2[0] * cos(pose_2->estimate()[2]) + diff_2[1] * sin(pose_2->estimate()[2]));
                // Diff dist projection to current pose direction
                vel1 *= fast_sigmoid(100
                                     * (diff_1[0] * cos(pose_1->estimate()[2])
                                        + diff_1[1] * sin(pose_1->estimate()[2])));
                vel2 *= fast_sigmoid(100
                                     * (diff_2[0] * cos(pose_2->estimate()[2])
                                        + diff_2[1] * sin(pose_2->estimate()[2])));

                double acc_v = (vel2 - vel1) * 2 / (dt_1->dt() + dt_2->dt());
                _error[0] =
                    penaltyBoundToInterval(acc_v, cfg_->robot.max_lin_acc, PENALTY_BOUND_EPSILON);

                const double omega1 = angle_diff_1 / dt_1->dt();
                const double omega2 = angle_diff_2 / dt_2->dt();
                const double acc_w = (omega2 - omega1) * 2 / (dt_1->dt() + dt_2->dt());
                _error[1] =
                    penaltyBoundToInterval(acc_w, cfg_->robot.max_rot_acc, PENALTY_BOUND_EPSILON);

#ifdef DEBUG_EDGE_ERROR
                if (_error[0] > 1e-1 || _error[1] > 1e-1) {
                    std::cout << "EdgeAcceleration"
                              << " error0:" << _error[0] << " error1:" << _error[1] << std::endl;
                }
#endif  //DEBUG_EDGE_ERROR

                //                if (fabs(_error[0]) > 0.1 ||
                //                    fabs(_error[1]) > 0.1) {
                //                    std::cout << "EdgeAcceleration pose_1:" << *pose_1
                //                              << " pose_2:" << *pose_2
                //                              << " pose_3:" << *pose_3
                //                              << " diff_1:(" << diff_1[0]
                //                              << ", " << diff_1[1]
                //                              << ", " << diff_1[2]
                //                              << ") diff_2:(" << diff_2[0]
                //                              << ", " << diff_2[1]
                //                              << ", " << diff_2[2]
                //                              << ") angle_diff_1:" << angle_diff_1
                //                              << " angle_diff_2:" << angle_diff_2
                //                              << " omega1:" << angle_diff_2
                //                              << " omega2:" << angle_diff_2
                //                              << " dt1:" << dt_1->dt()
                //                              << " dt2:" << dt_2->dt()
                //                              << " dist_1:" << dist_1
                //                              << " dist_1:" << dist_2
                //                              << " error0:" << _error[0]
                //                              << " error1:" << _error[1] << std::endl;
                //                }
            }

        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        };

        class EdgeAccelerationStart : public BaseMultiEdge<2, Eigen::Vector2d> {
        public:
            EdgeAccelerationStart() {
                _measurement << 0.0, 0.0;  // lin_vel, rot_vel
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

                // Use arc for calculation
                //            if (angle_diff != 0) {
                //                dist = fabs(angle_diff * (dist / (2 * sin(angle_diff / 2))));
                //            }

                double vel1 = _measurement[0];
                double vel2 = dist / dt->dt();
                vel2 *= fast_sigmoid(100
                                     * (diff[0] * cos(pose_1->estimate()[2])
                                        + diff[1] * sin(pose_1->estimate()[2])));

                double acc_v = (vel2 - vel1) / dt->dt();
                _error[0] =
                    penaltyBoundToInterval(acc_v, cfg_->robot.max_lin_acc, PENALTY_BOUND_EPSILON);

                const double omega1 = _measurement[1];
                const double omega2 = angle_diff / dt->dt();
                const double acc_w = (omega2 - omega1) / dt->dt();
                _error[1] =
                    penaltyBoundToInterval(acc_w, cfg_->robot.max_rot_acc, PENALTY_BOUND_EPSILON);

//                std::cout << "EdgeAccelerationStart pose_1:" << *pose_1
//                          << " pose_2:" << *pose_2
//                          << " diff:(" << diff[0]
//                          << ", " << diff[1]
//                          << ", " << diff[2]
//                          << ") dt:" << dt->dt()
//                          << " dist:" << dist
//                          << " error0:" << _error[0]
//                          << " error1:" << _error[1] << std::endl;
#ifdef DEBUG_EDGE_ERROR
                if (_error[0] > 1e-1 || _error[1] > 1e-1) {
                    std::cout << "EdgeAccelerationStart"
                              << " error0:" << _error[0] << " error1:" << _error[1] << std::endl;
                }
#endif  //DEBUG_EDGE_ERROR
            }

            void setInitialVelocity(const Eigen::Vector2f& vel_start) {
                _measurement = vel_start.cast<double>();
            }

        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        };

        class EdgeAccelerationGoal : public BaseMultiEdge<2, Eigen::Vector2d> {
        public:
            EdgeAccelerationGoal() {
                _measurement << 0.0, 0.0;  // lin_vel, rot_vel
                this->resize(3);
            }

            void computeError() override {
                // pose_1
                auto pose_1 = dynamic_cast<const VertexPose*>(_vertices[0]);
                // goal
                auto pose_2 = dynamic_cast<const VertexPose*>(_vertices[1]);
                auto dt = dynamic_cast<const VertexTimeDiff*>(_vertices[2]);

                // VELOCITY & ACCELERATION
                const Eigen::Vector3d diff = pose_2->estimate() - pose_1->estimate();

                double dist = diff.head(2).norm();
                const double angle_diff = g2o::normalize_theta(diff[2]);

                // Use arc for calculation
                //            if (angle_diff != 0) {
                //                dist = fabs(angle_diff * (dist / (2 * sin(angle_diff / 2))));
                //            }

                double vel1 = dist / dt->dt();
                vel1 *= fast_sigmoid(100
                                     * (diff[0] * cos(pose_1->estimate()[2])
                                        + diff[1] * sin(pose_1->estimate()[2])));
                double vel2 = _measurement[0];

                double acc_v = (vel2 - vel1) / dt->dt();
                _error[0] =
                    penaltyBoundToInterval(acc_v, cfg_->robot.max_lin_acc, PENALTY_BOUND_EPSILON);

                const double omega1 = angle_diff / dt->dt();
                const double omega2 = _measurement[1];
                const double acc_w = (omega2 - omega1) / dt->dt();
                _error[1] =
                    penaltyBoundToInterval(acc_w, cfg_->robot.max_rot_acc, PENALTY_BOUND_EPSILON);

//                std::cout << "EdgeAccelerationGoal pose_1:" << *pose_1
//                          << " pose_2:" << *pose_2
//                          << " diff:(" << diff[0]
//                          << ", " << diff[1]
//                          << ", " << diff[2]
//                          << ") dt:" << dt->dt()
//                          << " dist:" << dist
//                          << " error0:" << _error[0]
//                          << " error1:" << _error[1] << std::endl;
#ifdef DEBUG_EDGE_ERROR
                if (_error[0] > 1e-1 || _error[1] > 1e-1) {
                    std::cout << "EdgeAccelerationGoal"
                              << " error0:" << _error[0] << " error1:" << _error[1] << std::endl;
                }
#endif  //DEBUG_EDGE_ERROR
            }

            void setGoalVelocity(const Eigen::Vector2f& vel_goal) {
                _measurement = vel_goal.cast<double>();
            }

        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        };

    }  // namespace motion_planner
}  // namespace ninebot_algo
#endif  //MOTIONPLANNER_EDGEACCELERATION_HPP
