//
// Created by mazheng on 25-4-28.
//

#include "back_end/pso_smoother.hpp"
#include "utils/particle_swarm_optimization.hpp"
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/transform_datatypes.h>

using namespace uneven_planner;

SmoothTrackerParam PathCostCalculator::tracker_param_ = {};
Eigen::Vector3f PathCostCalculator::start_point_ = {0, 0, 0};
Eigen::Vector3f PathCostCalculator::end_point_ = {0, 0, 0};
float PathCostCalculator::step_size_ = 0.1f;

PSOSmoother::PSOSmoother() : speed_limit_(0.6, 0.5, 0.2) {}

void PSOSmoother::init(ros::NodeHandle &nh) {
    node_ = nh;
    //callback to receive trigger to smooth, input path: 1.some trajectories
}

bool PSOSmoother::smooth(nav_msgs::Path& path, const nav_msgs::Odometry& pose) {
    param_.max_vel_x = speed_limit_[0];
    param_.max_vel_theta = speed_limit_[1];
    param_.max_vel_x_backward = speed_limit_[2];

    std::vector<Eigen::Vector3f> init_path;
    for (auto pose : path.poses) {
        Eigen::Vector3f point;
        point[0] = pose.pose.position.x;
        point[1] = pose.pose.position.y;
        auto quat = pose.pose.orientation;
        tf::Matrix3x3 mat(tf::Quaternion(quat.x, quat.y, quat.z, quat.w));
        double yaw, pitch, roll;
        mat.getEulerYPR(yaw, pitch, roll);
        point[2] = yaw;
        init_path.emplace_back(point);
    }
    if (init_path.size() < 2) {
        m_init_path_ = init_path;
        std::cout << "Smooth error for init path size=." << init_path.size() << std::endl;
        return false;
    }

    int points_num = 0;
    refactorPath(init_path, m_init_path_, points_num);

    // init PathCostCalculator
    PathCostCalculator::setParam(param_);
    PathCostCalculator::setBoundaryCondition(m_init_path_.front(), m_init_path_.back());
    //
    // You can specify an InertiaWeightStrategy functor as
    // template parameter. There are ConstantWeight, LinearDecrease,
    // ExponentialDecrease1, ExponentialDecrease2, ExponentialDecrease3
    // available. (Default is ConstantWeight)
    //
    // You can additionally specify a Callback functor as template parameter.
    pso::ParticleSwarmOptimization<double, PathCostCalculator, pso::ExponentialDecrease1<double>>
            optimizer;

    // Set number of iterations as stop criterion.
    // Set it to 0 or negative for infinite iterations (default is 0).
    optimizer.setMaxIterations(MaxIterNum);

    // Set the minimum change of the x-values (particles) (default is 1e-6).
    // If the change in the current iteration is lower than this value, then
    // the optimizer stops minimizing.
    optimizer.setMinParticleChange(1e-2);

    // Set the minimum change of the function values (default is 1e-6).
    // If the change in the current iteration is lower than this value, then
    // the optimizer stops minimizing.
    optimizer.setMinFunctionChange(1e-2);

    // Set the number of threads used for evaluation (OpenMP only).
    // Set it to 0 or negative for auto detection (default is 1).
    optimizer.setThreads(MaxTheadNum);

    // Turn verbosity on, so the optimizer prints status updates after each
    // iteration.
    param_.verbose ? optimizer.setVerbosity(2) : optimizer.setVerbosity(0);

    // Set the bounds in which the optimizer should search.
    // Each column vector defines the (min, max) for each dimension  of the
    // particles.
    int col = 3 * ((int)m_init_path_.size() - 2);

    Eigen::MatrixXd bounds(2, col);
    int index = -1;
    for (int i = 1; i + 1 < m_init_path_.size(); ++i) {
        ++index;
        bounds(0, index) = m_init_path_[i][0] + param_.bound_x[0];  // lower bound x
        bounds(1, index) = m_init_path_[i][0] + param_.bound_x[1];  // upper bound x
        ++index;
        bounds(0, index) = m_init_path_[i][1] + param_.bound_y[0];  // lower bound y
        bounds(1, index) = m_init_path_[i][1] + param_.bound_y[1];  // upper bound y
        ++index;
        bounds(0, index) = m_init_path_[i][2] + param_.bound_theta[0];  // lower bound theta
        bounds(1, index) = m_init_path_[i][2] + param_.bound_theta[1];  // lower bound theta
    }

    //        std::cout << "bounds:" <<  bounds.transpose() << std::endl;

    // start the optimization with a particle count
    auto result = optimizer.minimize(bounds, std::max(50, 50 * (points_num - 2)));

    m_optimal_path_.clear();
    m_optimal_path_.emplace_back(m_init_path_.front());
    auto point_size = (int)result.xval.size() / 3;
    for (int i = 0; i < point_size; ++i) {
        m_optimal_path_.emplace_back(result.xval(3 * i), result.xval(3 * i + 1),
                                     result.xval(3 * i + 2));
    }
    m_optimal_path_.emplace_back(m_init_path_.back());
    if (param_.verbose) {
        std::string path_info;
        for (const auto& point : m_optimal_path_) {
            path_info += str_format("{%.2f,%.2f,%.2f} ", point[0], point[1], point[2]);
        }
        ROS_DEBUG("Optimal path:%s.", path_info.c_str());
    }

    // construct optimal trajectory
    for (int i = 1; i < m_optimal_path_.size(); ++i) {
        auto forward_time = PathCostCalculator::calculateSplineTime(
                m_optimal_path_[i - 1][0], m_optimal_path_[i - 1][1], m_optimal_path_[i - 1][2],
                m_optimal_path_[i][0], m_optimal_path_[i][1], m_optimal_path_[i][2], false);
        auto back_time = PathCostCalculator::calculateSplineTime(
                m_optimal_path_[i - 1][0], m_optimal_path_[i - 1][1], m_optimal_path_[i - 1][2],
                m_optimal_path_[i][0], m_optimal_path_[i][1], m_optimal_path_[i][2], true);
        auto max_check_num = (int)std::fmax(5.0f,
                                            hypot(m_optimal_path_[i][1] - m_optimal_path_[i - 1][1],
                                                  m_optimal_path_[i][0] - m_optimal_path_[i - 1][0])
                                            / PathStep
                                            + 1);
        std::unique_ptr<Eta3Spline> spline;
        if (forward_time < back_time) {
            spline = std::make_unique<Eta3Spline>(
                    m_optimal_path_[i - 1][0], m_optimal_path_[i - 1][1], m_optimal_path_[i - 1][2], 0,
                    0, m_optimal_path_[i][0], m_optimal_path_[i][1], m_optimal_path_[i][2], 0, 0);
            for (int j = 0; j <= max_check_num; ++j) {
                auto pos = spline->evaluate3D((float)j / (float)max_check_num);
                result_path_.emplace_back(pos.x(), pos.y(), pos.z());
            }
        } else {
            spline = std::make_unique<Eta3Spline>(
                    m_optimal_path_[i - 1][0], m_optimal_path_[i - 1][1],
                    m_optimal_path_[i - 1][2] + M_PI, 0, 0, m_optimal_path_[i][0],
                    m_optimal_path_[i][1], m_optimal_path_[i][2] + M_PI, 0, 0);
            for (int j = 0; j <= max_check_num; ++j) {
                auto pos = spline->evaluate3D((float)j / (float)max_check_num);
                pos[2] = (float)wrapToPi(pos[2] + M_PI);
                result_path_.emplace_back(pos.x(), pos.y(), pos.z());
            }
        }
    }

    // set smooth failed if optimal trajectory time cost too large
    bool is_success =
            result.fval < 1000;
    if (!is_success) {
        ROS_WARN("Smooth error, value=%.2f.", result.fval);
        return false;
    }

    std::string path_info;
    for (const auto& point : m_optimal_path_) {
        path_info += str_format("{%.2f,%.2f,%.2f} ", point[0], point[1], point[2]);
    }
    ROS_DEBUG("Optimal path:%s, value=%.2f, success %d.", path_info.c_str(), result.fval, is_success);

    path.header.stamp = ros::Time::now();
    path.header.frame_id = "world";
    path.poses.clear();
    for (auto point : result_path_) {
        geometry_msgs::PoseStamped pose;
        pose.header.stamp = ros::Time::now();
        pose.header.frame_id = "world";
        pose.pose.position.x = point[0];
        pose.pose.position.y = point[1];
        pose.pose.orientation = tf::createQuaternionMsgFromYaw(point[2]);
        path.poses.emplace_back(pose);
    }

    return is_success;
}

void
PSOSmoother::refactorPath(const std::vector<Eigen::Vector3f> &init_path, std::vector<Eigen::Vector3f> &refactor_path,
                          int& points_num) {
    // points num >= MinPointSize
    points_num = std::max(
            MinPointSize,
            static_cast<int>((init_path.front().head(2) - init_path.back().head(2)).norm() / StepSize)
            + 1);
    points_num = std::min(MaxPointSize, points_num);
    if (param_.verbose) ROS_DEBUG("Total path points size %d.", points_num);
    if (param_.verbose) {
        std::string init_path_info;
        for (const auto& point : init_path) {
            init_path_info += str_format("{%.2f,%.2f,%.2f},", point[0], point[1], point[2]);
        }
        ROS_DEBUG("Init path: %s.", init_path_info.c_str());
    }

    // remove pure rotate point or too close point
    refactor_path = init_path;
    for (auto it = refactor_path.begin() + 1; it + 1 != refactor_path.end();) {
        if (refactor_path.size() < 2) break;
        if ((*it - *(it - 1)).head(2).norm() < 1e-3f || (*it - *(it + 1)).head(2).norm() < 1e-3f) {
            //                if (verbose_) ALOGD("Erase too close point(%.2f,%.2f,%.2f).", it->x(), it->y(), it->z());
            it = refactor_path.erase(it);
        } else {
            ++it;
        }
    }
    if (param_.verbose) {
        std::string temp_path_info;
        for (const auto& point : refactor_path) {
            temp_path_info += str_format("{%.2f,%.2f,%.2f},", point[0], point[1], point[2]);
        }
        ROS_DEBUG("Init path after remove too close points: %s.", temp_path_info.c_str());
    }

    /// add point between max distance points
    while (refactor_path.size() < points_num) {
        // find max distance position
        int max_distance_index = 0;
        float max_distance = std::numeric_limits<float>::lowest();
        for (int i = 0; i + 1 < refactor_path.size(); ++i) {
            auto dis = (refactor_path[i].head(2) - refactor_path[i + 1].head(2)).norm();
            if (dis > max_distance) {
                max_distance = dis;
                max_distance_index = i;
            }
        }
        Eigen::Vector3f insert_point(
                (refactor_path[max_distance_index][0] + refactor_path[max_distance_index + 1][0]) / 2,
                (refactor_path[max_distance_index][1] + refactor_path[max_distance_index + 1][1]) / 2,
                (float)wrapToPi(
                        (refactor_path[max_distance_index][2] + refactor_path[max_distance_index + 1][2])
                        / 2));
        refactor_path.insert(refactor_path.begin() + max_distance_index + 1, insert_point);
        if (param_.verbose)
            ROS_DEBUG("Insert point(%.2f,%.2f,%.2f).", insert_point.x(), insert_point.y(),
                  insert_point.z());
    }

    /// remove min distance point
    while (refactor_path.size() > points_num) {
        int min_distance_index = 0;
        float min_distance = std::numeric_limits<float>::max();
        for (int i = 1; i + 1 < refactor_path.size(); ++i) {
            auto dis = std::fmin((refactor_path[i].head(2) - refactor_path[i - 1].head(2)).norm(),
                                 (refactor_path[i].head(2) - refactor_path[i + 1].head(2)).norm());
            if (dis < min_distance) {
                min_distance = dis;
                min_distance_index = i;
            }
        }
        if (param_.verbose) {
            ROS_DEBUG("Erase index %d, erase point(%.2f,%.2f,%.2f).", min_distance_index,
                  refactor_path[min_distance_index][0], refactor_path[min_distance_index][1],
                  refactor_path[min_distance_index][2]);
        }
        refactor_path.erase(refactor_path.begin() + min_distance_index);
        //            if(verbose_) {
        //                std::string path_info;
        //                for (const auto &point : refactor_path) {
        //                    path_info += str_format("{%.2f,%.2f,%.2f},", point[0], point[1], point[2]);
        //                }
        //                ALOGD("Remain path: %s.", path_info.c_str());
        //            }
    }

    // show path info
    if (param_.verbose) {
        std::string refactor_path_info;
        for (const auto& point : refactor_path) {
            refactor_path_info += str_format("{%.2f,%.2f,%.2f},", point[0], point[1], point[2]);
        }
        ROS_DEBUG("Refactor path: %s.", refactor_path_info.c_str());
    }
}