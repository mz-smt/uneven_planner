//
// Created by evanlin on 5/27/20.
//

#ifndef MOTIONPLANNER_VERTEXTIMEDIFF_HPP
#define MOTIONPLANNER_VERTEXTIMEDIFF_HPP

#include <g2o/core/base_vertex.h>

namespace ninebot_algo {
    namespace motion_planner {

        class VertexTimeDiff : public g2o::BaseVertex<1, double> {
        public:
            explicit VertexTimeDiff(bool fixed = false) {
                _estimate = 0.1f;
                setFixed(fixed);
            }

            explicit VertexTimeDiff(double dt, bool fixed = false) {
                _estimate = dt;
                setFixed(fixed);
            }

            ~VertexTimeDiff() override = default;

            double& dt() {
                return _estimate;
            }

            const double& dt() const {
                return _estimate;
            }

            void setToOriginImpl() override {
                _estimate = 0.1f;
            }

            void oplusImpl(const double* update) override {
                //            if (fabs(*update) > 1e-4) {
                //                std::cout << "VertexTimeDiff oplusImpl dt:" << _estimate
                //                          << " update:" << *update << std::endl;
                //            }
                _estimate += *update;
            }

            bool read(std::istream& is) override {
                is >> _estimate;
                return true;
            }

            bool write(std::ostream& os) const override {
                os << estimate();
                return os.good();
            }

            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        };

    }  // namespace motion_planner
}  // namespace ninebot_algo
#endif  //MOTIONPLANNER_VERTEXTIMEDIFF_HPP
