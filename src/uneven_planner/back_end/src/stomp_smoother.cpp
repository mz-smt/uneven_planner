//
// Created by mazheng on 25-5-14.
//

#include <nav_msgs/Path.h>
#include <tf/transform_datatypes.h>
#include "back_end/stomp_smoother.hpp"

using namespace uneven_planner;

STOMPSmoother::STOMPSmoother() {}

void STOMPSmoother::init(ros::NodeHandle &nh) {
    node_ = nh;
    result_path_pub_ = node_.advertise<nav_msgs::Path>("/stomp/result_path", 1);
}

void STOMPSmoother::setOdom(const Eigen::Vector3d& odom_pose, const Eigen::Quaterniond& quaternion_input) {
    cur_pose_ << odom_pose.x(), odom_pose.y(), odom_pose.z();
    q_ = quaternion_input;
}

bool STOMPSmoother::smooth(std::vector<Eigen::Vector3f> &path) {
    StompConfiguration config{};
    config.num_timesteps = path.size();
    config.num_dimensions = 3;
    config.control_cost_weight = -0.1f;
    config.max_rollouts = 20;
    config.num_iterations = 40;
    config.num_iterations_after_valid = 100;
    config.num_rollouts = 20;
    config.initialization_method = TrajectoryInitializations::LINEAR_INTERPOLATION;

    const std::vector<double> std_dev = { 1.0, 1.0, 1.0 };
    auto param_init = pathToMatrix(path);
    const std::vector<double> BIAS_THRESHOLD = { 0.10, 0.10, 0.10 };
    TaskPtr task(new DummyTask(param_init, BIAS_THRESHOLD, std_dev, cur_pose_, q_));
    stomp_ptr_ = std::make_shared<Stomp>(config, task);
    Eigen::MatrixXd param_result;
    Eigen::Vector3d first(path.front().x(), path.front().y(), path.front().z());
    Eigen::Vector3d last(path.back().x(), path.back().y(), path.back().z());
    auto result = stomp_ptr_->solve(first, last, param_result);
    path = matrixToPath(param_result);
    nav_msgs::Path result_path;
    result_path.header.stamp = ros::Time::now();
    result_path.header.frame_id = "world";
    geometry_msgs::PoseStamped temp_pose;
    temp_pose.header = result_path.header;
    result_path.poses.clear();
    for (const auto& point : path) {
        temp_pose.pose.position.x = point[0];
        temp_pose.pose.position.y = point[1];
        temp_pose.pose.orientation = tf::createQuaternionMsgFromYaw(point[2]);
        result_path.poses.push_back(temp_pose);
    }
    result_path_pub_.publish(result_path);
    return result;
}