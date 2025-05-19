//
// Created by evanlin on 5/28/20.
//

#ifndef MOTIONPLANNER_EDGEBASE_HPP
#define MOTIONPLANNER_EDGEBASE_HPP

#include <g2o/core/base_binary_edge.h>
#include <g2o/core/base_multi_edge.h>
#include <g2o/core/base_unary_edge.h>

#include <memory>

#include "utils/path_optimize/optimize_params.hpp"

//#define DEBUG_EDGE_ERROR

namespace ninebot_algo {
    namespace motion_planner {

        /**
         * @brief  Base edge connecting a single vertex in the optimization problem
         * @tparam D Dimension
         * @tparam E Element type
         * @tparam VertexXi
         */
        template<int D, typename E, typename VertexXi>
        class BaseUnaryEdge : public g2o::BaseUnaryEdge<D, E, VertexXi> {
        public:
            using typename g2o::BaseUnaryEdge<D, E, VertexXi>::ErrorVector;
            using g2o::BaseUnaryEdge<D, E, VertexXi>::computeError;

            BaseUnaryEdge() {
                _vertices[0] = NULL;
            }

            virtual ~BaseUnaryEdge() {
                if (_vertices[0]) {
                    _vertices[0]->edges().erase(this);
                }
            }

            virtual bool read(std::istream& is) {
                return true;
            }

            virtual bool write(std::ostream& os) const {
                return os.good();
            }

            void setParams(std::shared_ptr<OptimizeParams>& cfg) {
                cfg_ = cfg;
            }

        protected:
            using g2o::BaseUnaryEdge<D, E, VertexXi>::_error;
            using g2o::BaseUnaryEdge<D, E, VertexXi>::_vertices;

            std::shared_ptr<const OptimizeParams> cfg_;

        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        };

        /**
         * @brief Base edge connecting two vertices in the optimization problem
         * @tparam D Dimension
         * @tparam E Element type
         * @tparam VertexXi
         * @tparam VertexXj
         */
        template<int D, typename E, typename VertexXi, typename VertexXj>
        class BaseBinaryEdge : public g2o::BaseBinaryEdge<D, E, VertexXi, VertexXj> {
        public:
            using typename g2o::BaseBinaryEdge<D, E, VertexXi, VertexXj>::ErrorVector;
            using g2o::BaseBinaryEdge<D, E, VertexXi, VertexXj>::computeError;

            BaseBinaryEdge() {
                _vertices[0] = NULL;
                _vertices[1] = NULL;
            }

            virtual ~BaseBinaryEdge() {
                if (_vertices[0]) {
                    _vertices[0]->edges().erase(this);
                }
                if (_vertices[1]) {
                    _vertices[1]->edges().erase(this);
                }
            }

            virtual bool read(std::istream& is) {
                return true;
            }

            virtual bool write(std::ostream& os) const {
                return os.good();
            }

            void setParams(std::shared_ptr<OptimizeParams>& cfg) {
                cfg_ = cfg;
            }

        protected:
            using g2o::BaseBinaryEdge<D, E, VertexXi, VertexXj>::_error;
            using g2o::BaseBinaryEdge<D, E, VertexXi, VertexXj>::_vertices;

            std::shared_ptr<const OptimizeParams> cfg_;

        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        };

        /**
         * @brief Base edge connecting two vertices in the optimization problem
         * @tparam D Dimension
         * @tparam E Element type
         */
        template<int D, typename E>
        class BaseMultiEdge : public g2o::BaseMultiEdge<D, E> {
        public:
            using typename g2o::BaseMultiEdge<D, E>::ErrorVector;
            using g2o::BaseMultiEdge<D, E>::computeError;

            BaseMultiEdge() {}

            virtual ~BaseMultiEdge() {
                for (std::size_t i = 0; i < _vertices.size(); ++i) {
                    if (_vertices[i]) {
                        _vertices[i]->edges().erase(this);
                    }
                }
            }

            virtual bool read(std::istream& is) {
                return true;
            }

            virtual bool write(std::ostream& os) const {
                return os.good();
            }

            virtual void resize(size_t size) {
                g2o::BaseMultiEdge<D, E>::resize(size);

                for (std::size_t i = 0; i < _vertices.size(); ++i)
                    _vertices[i] = NULL;
            }

            void setParams(std::shared_ptr<OptimizeParams>& cfg) {
                cfg_ = cfg;
            }

        protected:
            using g2o::BaseMultiEdge<D, E>::_error;
            using g2o::BaseMultiEdge<D, E>::_vertices;

            std::shared_ptr<const OptimizeParams> cfg_;

        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        };

    }  // namespace motion_planner
}  // namespace ninebot_algo

#endif  //MOTIONPLANNER_EDGEBASE_HPP
