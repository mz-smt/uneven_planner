//
// Created by evanlin on 5/28/20.
//

#ifndef MOTIONPLANNER_EDGEKINEMATICS_HPP
#define MOTIONPLANNER_EDGEKINEMATICS_HPP

#include <cmath>

#include "edge_base.hpp"
#include "optimize_params.hpp"
#include "penalty_bound.hpp"
#include "vertex_pose.hpp"
#include "vertex_time_diff.hpp"

namespace ninebot_algo {
    namespace motion_planner {

        class EdgeKinematics : public BaseBinaryEdge<2, double, VertexPose, VertexPose> {
        public:
            EdgeKinematics() {
                _measurement = 0.0f;
            }

            void computeError() override {
                auto pose_1 = dynamic_cast<const VertexPose*>(_vertices[0]);
                auto pose_2 = dynamic_cast<const VertexPose*>(_vertices[1]);

                const Eigen::Vector3d pose_diff = pose_2->estimate() - pose_1->estimate();

                _error[0] = fabs(
                    (cos(pose_1->estimate()[2]) + cos(pose_2->estimate()[2])) * pose_diff[1]
                    - (sin(pose_1->estimate()[2]) + sin(pose_2->estimate()[2])) * pose_diff[0]);

                Eigen::Vector2d angle_vec(cos(pose_1->estimate()[2]), sin(pose_1->estimate()[2]));
                double angle_constraint = pose_diff.head(2).dot(angle_vec);
                _error[1] = penaltyBoundFromBelow(angle_constraint, 0, 0);

//                if (_error[0] > 1e-1 ||
//                    _error[1] > 1e-1) {
//                    std::cout << "EdgeKinematics pose1:" << *pose_1
//                              << " pose2:" << *pose_2
//                              << " angle_constraint:" << angle_constraint
//                              << " error0:" << _error[0]
//                              << " error1:" << _error[1] << std::endl;
//                }
#ifdef DEBUG_EDGE_ERROR
                if (_error[0] > 1e-1 || _error[1] > 1e-1) {
                    std::cout << "EdgeKinematics"
                              << " error0:" << _error[0] << " error1:" << _error[1] << std::endl;
                }
#endif  //DEBUG_EDGE_ERROR
            }

            void linearizeOplus() override {
                auto conf1 = dynamic_cast<const VertexPose*>(_vertices[0]);
                auto conf2 = dynamic_cast<const VertexPose*>(_vertices[1]);

                Eigen::Vector2d deltaS = (conf2->estimate() - conf1->estimate()).head(2);

                double cos1 = cos(conf1->estimate()[2]);
                double cos2 = cos(conf2->estimate()[2]);
                double sin1 = sin(conf1->estimate()[2]);
                double sin2 = sin(conf2->estimate()[2]);
                double aux1 = sin1 + sin2;
                double aux2 = cos1 + cos2;

                double dd_error_1 = deltaS[0] * cos1;
                double dd_error_2 = deltaS[1] * sin1;
                double dd_dev = penaltyBoundFromBelowDerivative(dd_error_1 + dd_error_2, 0, 0);

                double dev_nh_abs = g2o::sign(aux2 * deltaS[1] - aux1 * deltaS[0]);

                // conf1
                _jacobianOplusXi(0, 0) = aux1 * dev_nh_abs;  // nh x1
                _jacobianOplusXi(0, 1) = -aux2 * dev_nh_abs;  // nh y1
                _jacobianOplusXi(1, 0) = -cos1 * dd_dev;  // drive-dir x1
                _jacobianOplusXi(1, 1) = -sin1 * dd_dev;  // drive-dir y1
                _jacobianOplusXi(0, 2) = (-dd_error_2 - dd_error_1) * dev_nh_abs;  // nh angle
                _jacobianOplusXi(1, 2) =
                    (-sin1 * deltaS[0] + cos1 * deltaS[1]) * dd_dev;  // drive-dir angle1

                // conf2
                _jacobianOplusXj(0, 0) = -aux1 * dev_nh_abs;  // nh x2
                _jacobianOplusXj(0, 1) = aux2 * dev_nh_abs;  // nh y2
                _jacobianOplusXj(1, 0) = cos1 * dd_dev;  // drive-dir x2
                _jacobianOplusXj(1, 1) = sin1 * dd_dev;  // drive-dir y2
                _jacobianOplusXj(0, 2) =
                    (-sin2 * deltaS[1] - cos2 * deltaS[0]) * dev_nh_abs;  // nh angle
                _jacobianOplusXj(1, 2) = 0;  // drive-dir angle1
            }

        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        };

    }  // namespace motion_planner
}  // namespace ninebot_algo

#endif  //MOTIONPLANNER_EDGEKINEMATICS_HPP
