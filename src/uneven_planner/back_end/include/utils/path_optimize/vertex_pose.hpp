//
// Created by evanlin on 5/27/20.
//

#ifndef MOTIONPLANNER_VERTEXPOSE_HPP
#define MOTIONPLANNER_VERTEXPOSE_HPP

#include <g2o/core/base_vertex.h>
#include <g2o/stuff/misc.h>

#include <iostream>

namespace ninebot_algo {
    namespace motion_planner {

        class VertexPose : public g2o::BaseVertex<3, Eigen::Vector3d> {
        public:
            explicit VertexPose(bool fixed = false) {
                _estimate.setZero();
                setFixed(fixed);
            }

            explicit VertexPose(const Eigen::Vector3d& pose, bool fixed = false) {
                _estimate = pose;
                setFixed(fixed);
            }

            explicit VertexPose(const Eigen::Vector3f& pose, bool fixed = false) {
                _estimate = pose.cast<double>();
                setFixed(fixed);
            }

            explicit VertexPose(double x, double y, double theta, bool fixed = false) {
                _estimate.x() = x;
                _estimate.y() = y;
                _estimate.z() = theta;
                setFixed(fixed);
            }

            ~VertexPose() override = default;

            friend std::ostream& operator<<(std::ostream& out, const VertexPose& vertex_pose) {
                out << "(" << vertex_pose._estimate[0] << ", " << vertex_pose._estimate[1] << ", "
                    << vertex_pose._estimate[2] << ")";
                return out;
            }

            Eigen::Vector3d& pose() {
                return _estimate;
            }

            void setToOriginImpl() override {
                _estimate.setZero();
            }

            void oplusImpl(const double* update) override {
                //                if (fabs(update[0]) > 1e-2 ||
                //                    fabs(update[1]) > 1e-2 ||
                //                    fabs(update[2]) > 1e-2) {
                //                    std::cout << "VertexPose oplusImpl" << *this
                //                              << " update:(" << update[0]
                //                              << ", " << update[1]
                //                              << ", " << update[2] << ")" << std::endl;
                //                }
                _estimate.coeffRef(0) += update[0];
                _estimate.coeffRef(1) += update[1];
                _estimate[2] = g2o::normalize_theta(_estimate[2] + update[2]);
                //            _estimate[0] += update[0] * 1e6;
                //            _estimate[1] += update[1] * 1e6;
                //            _estimate[2] += g2o::normalize_theta(update[2]) * 1e6;
            }

            bool read(std::istream& is) override {
                is >> _estimate.x() >> _estimate.y() >> _estimate.z();
                return true;
            }

            bool write(std::ostream& os) const override {
                os << _estimate.x() << " " << _estimate.y() << " " << _estimate.z();
                return os.good();
            }

            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        };
    }  // namespace motion_planner
}  // namespace ninebot_algo

#endif  //MOTIONPLANNER_VERTEXPOSE_HPP
