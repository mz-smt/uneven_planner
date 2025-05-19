//
// Created by evanlin on 5/28/20.
//

#ifndef MOTIONPLANNER_EDGETIMEOPTIMAL_HPP
#define MOTIONPLANNER_EDGETIMEOPTIMAL_HPP

#include "edge_base.hpp"
#include "optimize_params.hpp"
#include "penalty_bound.hpp"
#include "vertex_pose.hpp"
#include "vertex_time_diff.hpp"

namespace ninebot_algo {
    namespace motion_planner {

        class EdgeTimeOptimal : public BaseUnaryEdge<1, double, VertexTimeDiff> {
        public:
            EdgeTimeOptimal() {
                _measurement = 0.0f;
            }

            void computeError() override {
                _error[0] = dynamic_cast<const VertexTimeDiff*>(_vertices[0])->dt();
//                _error[0] = penaltyBoundFromAbove(dynamic_cast<const VertexTimeDiff *>(_vertices[0])->dt(), 0.3, 0.0);
//                std::cout << "EdgeTimeOptimal time_diff error:" << _error[0] << std::endl;
#ifdef DEBUG_EDGE_ERROR
                if (_error[0] > 1e-1) {
                    std::cout << "EdgeTimeOptimal"
                              << " error0:" << _error[0] << std::endl;
                }
#endif  //DEBUG_EDGE_ERROR
            }

            void linearizeOplus() override {
                _jacobianOplusXi(0, 0) = 1;
            }

        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        };
    }  // namespace motion_planner
}  // namespace ninebot_algo

#endif  //MOTIONPLANNER_EDGETIMEOPTIMAL_HPP
