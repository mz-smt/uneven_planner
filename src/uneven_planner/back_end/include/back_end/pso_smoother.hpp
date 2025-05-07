//
// Created by mazheng on 25-4-28.
//

#ifndef SRC_PSO_SMOOTHER_HPP
#define SRC_PSO_SMOOTHER_HPP

#include <Eigen/Core>
#include <utils/eta3_spline.hpp>
#include <memory>
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include "utils/math_utils.hpp"

namespace uneven_planner {

    struct SmoothTrackerParam {
        SmoothTrackerParam()
                : max_vel_x(0.4f),
                  max_vel_x_backward(0.2f),
                  max_vel_theta(0.5f),
                  bound_x(-0.5f, 0.5f),
                  bound_y(-0.5f, 0.5f),
                  bound_theta(-M_PI, M_PI),
                  max_backward_dis(0.3f),
                  weight_penalty_backward(500.0f),
                  weight_penalty_gear_switch(0.5f),
                  verbose(true) {}
        float max_vel_x;
        float max_vel_x_backward;
        float max_vel_theta;
        Eigen::Vector2f bound_x;  // lower,upper
        Eigen::Vector2f bound_y;
        Eigen::Vector2f bound_theta;

        float max_backward_dis;
        float weight_penalty_backward;
        float weight_penalty_gear_switch;

        bool verbose;
    };

    class PathCostCalculator {
    public:
        PathCostCalculator() = default;

        static void setParam(const SmoothTrackerParam& param) {
            tracker_param_ = param;
            //            ALOGD("Param max vel %.2f, min vel %.2f, max angular vel %.2f.",
            //                  max_forward_linear_vel_, max_backward_linear_vel_, max_angular_vel_);
        }

        /**
         * @param start_point_body start point must in body frame
         * @param end_point_body end point must in body frame
         * */
        static void setBoundaryCondition(const Eigen::Vector3f& start_point_body,
                                         const Eigen::Vector3f& end_point_body) {
            start_point_ = start_point_body;
            end_point_ = end_point_body;
            //            ALOGD("SetBoundaryCondition start(%.2f,%.2f,%.2f), end(%.2f,%.2f,%.2f).",
            //                  start_point_[0], start_point_[1], start_point_[2],
            //                  end_point_[0], end_point_[1], end_point_[2]);
        }

        /**
         * @brief calculate spline time cost with collision penalty and backward distance penalty.
         * @param xa,ya,theta_a  start point
         * @param xb,yb,theta_b  end point
         * @param is_reverse True:spline backward move. False: spline forward move.
         * */
        static double calculateSplineTime(float xa, float ya, float theta_a, float xb, float yb,
                                          float theta_b, bool is_reverse) {
            double spline_cost = 0;
            auto max_linear_vel = is_reverse ? std::fabs(tracker_param_.max_vel_x_backward)
                                             : std::fabs(tracker_param_.max_vel_x);
            auto max_angular_vel = std::fabs(tracker_param_.max_vel_theta);
            if (is_reverse) {
                theta_a += M_PI;
                theta_b += M_PI;
            }
            auto spline =
                    std::make_unique<Eta3Spline>(xa, ya, theta_a, 0, 0, xb, yb, theta_b, 0, 0);
            auto point_dist = hypot(yb - ya, xb - xa);
            auto max_check_num = (int)std::fmax(5.0f, point_dist / step_size_ + 1);

            Eigen::Vector3f last_position;
            for (int j = 0; j <= max_check_num; ++j) {
                auto pos = spline->evaluate3D((float)j / (float)max_check_num);
                if (is_reverse) pos[2] = (float)wrapToPi(pos[2] + M_PI);

                /// calculate time
                if (j != 0) {
                    auto dist_cost = (pos.head(2) - last_position.head(2)).norm() / max_linear_vel;
                    auto angle_cost =
                            std::fabs(wrapToPi(pos(2) - last_position(2))) / max_angular_vel;
                    spline_cost += std::fmax(dist_cost, angle_cost);
                }
                last_position = std::move(pos);
            }

            /// penalty backward distance
            if (is_reverse && point_dist > tracker_param_.max_backward_dis)
                spline_cost += tracker_param_.weight_penalty_backward;
            /// additional penalty
            return spline_cost;
        }

        // TODO: Optimize: use precise time considering acc and dcc
        /**
         * @brief Calculate trajectory time cost
         * @param path_points Trajectory point(x,y,theta) in a col
         * @return Time cost
         * */
        template<typename Derived>
        double operator()(const Eigen::MatrixBase<Derived>& path_points) const {
            // point(x,y,theta)
            int points_size = (int)path_points.size() / 3;
            assert(points_size >= 1);
            assert(path_points.size() % 3 == 0);

            float xa, ya, theta_a, xb, yb, theta_b;
            double spline_cost = 0;
            int8_t gear_switch = 0;
            for (int i = 0; i <= points_size; ++i) {
                // point a
                if (i == 0) {
                    xa = start_point_[0];
                    ya = start_point_[1];
                    theta_a = start_point_[2];
                } else {
                    xa = xb;
                    ya = yb;
                    theta_a = theta_b;
                }

                // point b
                if (i == points_size) {
                    xb = end_point_[0];
                    yb = end_point_[1];
                    theta_b = end_point_[2];
                } else {
                    auto index = 3 * i;
                    xb = path_points(index);
                    yb = path_points(++index);
                    theta_b = path_points(++index);
                }
                // min(spline_forward_value, spline_back_value)
                auto spline_forward_cost =
                        calculateSplineTime(xa, ya, theta_a, xb, yb, theta_b, false);
                auto spline_back_cost = calculateSplineTime(xa, ya, theta_a, xb, yb, theta_b, true);
                //                spline_cost += std::min(spline_forward_cost, spline_back_cost);
                auto&& cost_forward =
                        (gear_switch < 0
                         ? (spline_forward_cost + tracker_param_.weight_penalty_gear_switch)
                         : spline_forward_cost);
                auto&& cost_back =
                        (gear_switch > 0
                         ? (spline_back_cost + tracker_param_.weight_penalty_gear_switch)
                         : spline_back_cost);
                gear_switch = cost_forward <= cost_back ? 1 : -1;
                spline_cost += gear_switch > 0 ? cost_forward : cost_back;
            }
            //            ALOGD("total_spline_cost: %.2f.", spline_cost);
            return spline_cost;
        }

    private:
        static Eigen::Vector3f start_point_;
        static Eigen::Vector3f end_point_;

    public:
        static SmoothTrackerParam tracker_param_;
        static float step_size_;
    };

    class PSOSmoother {
    public:
        explicit PSOSmoother();

        void init(ros::NodeHandle& nh);

        bool smooth(nav_msgs::Path& path, const nav_msgs::Odometry& pose);

    private:
        void refactorPath(const std::vector<Eigen::Vector3f>& init_path, std::vector<Eigen::Vector3f>& refactor_path,
                          int& points_num);

        ros::NodeHandle node_;

        SmoothTrackerParam param_;

        std::vector<Eigen::Vector3f> m_init_path_;
        std::vector<Eigen::Vector3f> m_optimal_path_;
        std::vector<Eigen::Vector3f> result_path_;

        const float StepSize = 0.5f;
        const int MinPointSize = 3;
        const int MaxPointSize = 5;
        const float PathStep = 0.1f;
        const int MaxIterNum = 20;
        const int MaxTheadNum = 4;

        Eigen::Vector3f speed_limit_;
    };
}



#endif //SRC_PSO_SMOOTHER_HPP
