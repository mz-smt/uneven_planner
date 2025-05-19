//
// Created by evanlin on 5/28/20.
//

#include "utils/path_optimize/path_optimizer.hpp"

#include <g2o/core/factory.h>

#include <iostream>
#include <mutex>
#include <utility>

#include "utils/path_optimize/edge_acceleration.hpp"
#include "utils/path_optimize/edge_kinematics.hpp"
#include "utils/path_optimize/edge_time_optimal.hpp"
#include "utils/path_optimize/edge_velocity.hpp"
#include "utils/path_optimize/optimize_params.hpp"
#include "utils/path_optimize/timed_elastic_band.hpp"
#include "utils/path_optimize/vertex_pose.hpp"
#include "utils/path_optimize/vertex_time_diff.hpp"
#include "utils/path_optimize/edge_slip.hpp"
#include "utils/math_util.hpp"
#include <nav_msgs/Path.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_datatypes.h>

using namespace std;
using namespace ninebot_algo;
using namespace uneven_planner;
using namespace ninebot_algo::motion_planner;

PathOptimizer::PathOptimizer()
    :  path_update_(false), fix_final_goal_(false) {
    cfg_ = std::make_shared<OptimizeParams>();
    /* 1.Init DM */
    //    cfg_ = std::make_shared<OptimizeParams>();

    /* 2.Init optimize graph types */
    // For multi-thread
    static std::once_flag flag;
    std::call_once(flag, &PathOptimizer::registerG2OTypes, this);
    //    registerG2OTypes();

    /* 3.Init Optimizer */
    /* 3.1 line solver */
    std::unique_ptr<LinearSolver> linearSolver(
        new LinearSolver());  // see typedef in optimization.h
    linearSolver->setBlockOrdering(true);
    /* 3.2 block solver */
    std::unique_ptr<BlockSolver> blockSolver(new BlockSolver(std::move(linearSolver)));
    /* 3.3 algorithm */
    auto solver = new g2o::OptimizationAlgorithmLevenberg(std::move(blockSolver));
    /* 3.4 optimizer */
    optimizer_ = std::make_shared<g2o::SparseOptimizer>();
    optimizer_->setAlgorithm(solver);
    optimizer_->initMultiThreading();

    /* 4.Init variables */
    vel_start_ << 0.0f, 0.0f;
    vel_goal_ << 0.0f, 0.0f;
    best_traj_.clear();
    cur_pose_ << 0.0f, 0.0f, 0.0f;
    cmd_ << 0.0f, 0.0f;
    ref_traj_.clear();
    nearest_idx_ = 0;

    /* 5.Path vertex */
    teb_ = std::make_unique<TimedElasticBand>();

}

PathOptimizer::~PathOptimizer() {
    clearGraph();
}

void PathOptimizer::setReferenceTrajectory(const std::vector<Eigen::Vector3f>& traj) {
    ref_traj_ = traj;
    nearest_idx_ = 0;

    /* Parse path to Vertex */
    best_traj_.clear();

    teb_->clearVertexSequences();
    path_update_ = true;
    fix_final_goal_ = false;
    next_nearest_index_ = 0;
    std::cout << "debug path optimizer set reference path size: " << ref_traj_.size() << std::endl;
}

PlannerStatus PathOptimizer::makePlan(const Eigen::Vector3f& pose, const Eigen::Vector2f& speed) {
    cur_pose_ = pose;
    cmd_ << 0.0f, 0.0f;

    if (ref_traj_.empty()) {
        return PlannerStatus::Goal_Reached;
    }
    nearest_idx_ = getNearestWayPoint(ref_traj_, pose, nearest_idx_, ref_traj_.size() - 1, 2.0f);
    auto dist_to_path = (cur_pose_ - ref_traj_.at(nearest_idx_)).head(2).norm();
    if (dist_to_path > PathLength) {
//        ALOGD("dist to path too far: %f", dist_to_path);
        return PlannerStatus::Error;
    }
    float remain_dist = 0.0f;
    goal_index_ = nearest_idx_;
    for (int i = nearest_idx_; i < ref_traj_.size() - 1; i++) {
        goal_index_ = i + 1;
        remain_dist += (ref_traj_.at(i) - ref_traj_.at(i + 1)).head(2).norm();
        if (remain_dist > 2.0f) {
            break;
        }
    }
//    ALOGD(
//        "index: %d, goal index: %d, current pose: (%f, %f, %f), collision dist: %f, risk cost: %f",
//        nearest_idx_, goal_index_, cur_pose_[0], cur_pose_[1], cur_pose_[2], collision_dist,
//        risk_dist);
    if (path_update_) {
        path_update_ = false;
        auto init_path = getInitPath();
        teb_->initBandSequence(init_path, cur_pose_, cfg_->robot.max_lin_vel_forward,
                               cfg_->robot.max_rot_vel);
        if ((init_path.back() - ref_traj_.back()).norm() < 1e-4) {
            teb_->setBackPoseFixed();
            fix_final_goal_ = true;
//            ALOGD("init path set fix");
        }
    } else {
        if (!fix_final_goal_) {
            auto next_path = getNextBackPath();
            if (!next_path.empty()) {
                teb_->addPosesAndTimeDiffs(next_path);
                if ((next_path.back() - ref_traj_.back()).norm() < 1e-4) {
                    teb_->setBackPoseFixed();
                    fix_final_goal_ = true;
//                    ALOGD("next path set fix");
                }
            }
        }
    }
    if (!teb_->isInit()) {
        //        teb_->initBandSequence(path, cfg_->robot.max_lin_vel_forward, cfg_->robot.max_rot_vel);
//        ALOGD("Teb is not init.");
        return PlannerStatus::Error;
    } else {
        /* Goal Reached */
        Eigen::Vector3f goal_diff = pose - ref_traj_.back();
        auto dist = goal_diff.head(2).norm();

        //        cout << "backPose: (" << teb_->backPose()[0]
        //             << ", " << teb_->backPose()[1]
        //             << ", " << teb_->backPose()[2]
        //             << ")  (" << pose[0]
        //             << ", " << pose[1]
        //             << ", " << pose[2]
        //             << ")" << endl;
        //        cout << "Dist: " << goal_diff.head(2).squaredNorm() << "  theta:" << goal_diff[2] << endl;
//        ALOGD("Dist: %f, theta: %f", dist, wrapToPi(goal_diff[2]));
        if (dist < cfg_->goal_tolerance.xy_goal_tolerance
            && fabs(uneven_planner::wrapToPi(goal_diff[2])) < cfg_->goal_tolerance.yaw_goal_tolerance
            && fabs(cmd_[0]) < 0.2 && fabs(cmd_[1]) < 0.1) {
            // Goal reach
            return PlannerStatus::Goal_Reached;
        }

        /* Test1: Directly prune TEB */
        teb_->updateAndPruneTEB(pose);
    }

    /* Set velocity to build graph */
    vel_start_ = speed;
    std::cout << "debug path optimizer start to graph optimize" << std::endl;
    /* Optimize */
    if (!graphOptimize(cfg_->optimize.num_inner_iterations, cfg_->optimize.num_outer_iterations)) {
//        ALOGD("graphOptimize failed");
        return PlannerStatus::Failed;
    }
    std::vector<Eigen::Vector3f> colors_ = {
            {1.0, 0.0, 0.0},  // 红
            {0.0, 1.0, 0.0},  // 绿
            {0.0, 0.0, 1.0},  // 蓝
            {1.0, 1.0, 0.0},  // 黄
            {1.0, 0.0, 1.0}   // 紫
    };
    visualization_msgs::MarkerArray arr;
    ros::Time now = ros::Time::now();
    for (size_t i = 0; i < debug_trajs_.size(); i++) {
        visualization_msgs::Marker line;
        line.header.frame_id = "world";
        line.header.stamp    = now;
        line.ns              = "traj_bundle";
        line.id              = i;
        line.type            = visualization_msgs::Marker::LINE_STRIP;
        line.action          = visualization_msgs::Marker::ADD;
        line.scale.x         = 0.001;
        auto c = colors_[i % colors_.size()];
        line.color.r = c[0]; line.color.g = c[1]; line.color.b = c[2]; line.color.a = 0.8;
        for (auto& p : debug_trajs_[i]) {
            geometry_msgs::Point pt; pt.x=p.x(); pt.y=p.y(); pt.z=0.0;
            line.points.push_back(pt);
        }
        arr.markers.push_back(line);

        // —— 2. 为每个点添加箭头 ——
        for (size_t j = 0; j < debug_trajs_[i].size(); ++j) {
            visualization_msgs::Marker arrow;
            arrow.header.frame_id = "world";
            arrow.header.stamp    = now;
            arrow.ns              = "traj_orient";
            arrow.id              = i * 1000 + j;  // 保证与 LINE_STRIP 不冲突 :contentReference[oaicite:7]{index=7}
            arrow.type            = visualization_msgs::Marker::ARROW;
            arrow.action          = visualization_msgs::Marker::ADD;
            // 尺寸：x=杆长，y=杆粗，z=头部粗
            arrow.scale.x         = 0.03;
            arrow.scale.y         = 0.003;
            arrow.scale.z         = 0.005;
            arrow.color           = line.color;     // 同路径线颜色，可自行调整透明度等
            // 位姿：位置与点重合，方向使用预先计算的四元数
            arrow.pose.position.x    = debug_trajs_[i][j].x();
            arrow.pose.position.y    = debug_trajs_[i][j].y();
            arrow.pose.position.z    = 0.0;
            arrow.pose.orientation   = tf::createQuaternionMsgFromYaw(debug_trajs_[i][j].z());
            arr.markers.push_back(arrow);
        }
    }
    teb_debug_pub_.publish(arr);

    /* Size check */
    if (teb_->poses().size() < 3) {
//        ALOGD("PathOptimizer::plan() TEB poses size < 3!");
        return PlannerStatus::Error;
    }
    nav_msgs::Path result_path;
    result_path.header.frame_id = "world";
    result_path.header.stamp = ros::Time::now();
    for (const auto& pt : teb_->poses()) {
        auto point = pt->pose().cast<float>();
        geometry_msgs::PoseStamped temp_pose;
        temp_pose.header.frame_id = "world";
        temp_pose.header.stamp = ros::Time::now();
        temp_pose.pose.position.x = point.x();
        temp_pose.pose.position.y = point.y();
        temp_pose.pose.orientation = tf::createQuaternionMsgFromYaw(point.z());
        result_path.poses.emplace_back(temp_pose);
    }
    teb_result_pub_.publish(result_path);

    /* Update cmd */
    if (!calcCmd()) {
        return PlannerStatus::Error;
    }

    //    cout << "Optimal cmd:" << cmd_[0] << ", " << cmd_[1] << endl;

    return PlannerStatus::Normal;
}

//Trajectory PathOptimizer::getOptimalTrajectory() {
//    return Trajectory(best_traj_);
//}

Eigen::Vector2f PathOptimizer::getOptimalControlCmd() {
    //    cout << "Pose Diff:" << teb_->pose(0) - teb_->pose(1)  << endl;
    //    cout << "Time Diff:" << teb_->timeDiff(0) << endl;
    //    cout << "cmd:" << cmd_[0] << " " << cmd_[1] << endl;
    return cmd_;
}

Eigen::Vector3f PathOptimizer::getCurrentTarget() {
    Eigen::Vector3f temp;
    if (!teb_->poses().empty()) {
        temp = teb_->backPose().cast<float>();
    }
    return temp;
}

void PathOptimizer::setGoalVel(Eigen::Vector2f vel_goal) {
    vel_goal_ = std::move(vel_goal);
}

/* v/w/t */
std::vector<Eigen::Vector3f> PathOptimizer::getVelocityProfile() const {
    int n = teb_->poses().size();
    std::vector<Eigen::Vector3f> vel_profile(n + 1);

    float delta_t = 0.0f;
    vel_profile.front() = {vel_start_[0], vel_start_[1], delta_t};

    Eigen::Vector2f extract_cmd;
    for (int i = 1; i < n; ++i) {
        extract_cmd =
            extractVelocity(teb_->pose(i - 1), teb_->pose(i), (float)teb_->timeDiff(i - 1));

        delta_t += (float)teb_->timeDiff(i - 1);
        vel_profile[i] = {extract_cmd[0], extract_cmd[1], delta_t};
    }

    delta_t += (float)teb_->backTimeDiff();
    vel_profile.back() = {vel_goal_[0], vel_goal_[1], delta_t};

    return vel_profile;
}

/* --------> Private Method <--------*/

void PathOptimizer::registerG2OTypes() {
    g2o::Factory* factory = g2o::Factory::instance();
    factory->registerType("OPTIMIZER_VERTEX_POSE", new g2o::HyperGraphElementCreator<VertexPose>);
    factory->registerType("OPTIMIZER_VERTEX_TIME_DIFF",
                          new g2o::HyperGraphElementCreator<VertexTimeDiff>);

    factory->registerType("OPTIMIZER_EDGE_TIME_OPTIMAL",
                          new g2o::HyperGraphElementCreator<EdgeTimeOptimal>);
    factory->registerType("OPTIMIZER_EDGE_VELOCITY",
                          new g2o::HyperGraphElementCreator<EdgeVelocity>);
    factory->registerType("OPTIMIZER_EDGE_ACCELERATION",
                          new g2o::HyperGraphElementCreator<EdgeAcceleration>);
    factory->registerType("OPTIMIZER_EDGE_ACCELERATION_START",
                          new g2o::HyperGraphElementCreator<EdgeAccelerationStart>);
    factory->registerType("OPTIMIZER_EDGE_ACCELERATION_GOAL",
                          new g2o::HyperGraphElementCreator<EdgeAccelerationGoal>);
    factory->registerType("OPTIMIZER_EDGE_KINEMATICS",
                          new g2o::HyperGraphElementCreator<EdgeKinematics>);
}

bool PathOptimizer::graphOptimize(int iterations_inner_loop, int iterations_outer_loop) {
    /* Loop optimize */
    debug_trajs_.clear();
    std::vector<Eigen::Vector3f> temp_traj;
    for (int i = 0; i < iterations_outer_loop; ++i) {
        std::cout << "optimize iteration: " << i << endl;
        /* Auto resize teb */
        teb_->autoResize(cfg_->optimize.time_step);

        /* Verify teb_ */
        // todo: if goal is close to start teb auto resize may lose poses
        if (!teb_->isInit() || teb_->poses().size() < 2) {
            clearGraph();
//            ALOGD("Init poses size too small!");
            return false;
        }

        /* Build graph */
        if (!buildGraph()) {
            clearGraph();
            cout << "buildGraph failed!" << endl;
//            ALOGD("buildGraph failed!");
            return false;
        }
        std::cout << "optimize finish build graph start to optimize " << endl;
        /* Call optimize */
        optimizer_->setVerbose(false);  // Release change to false
        optimizer_->initializeOptimization();
        int iter = optimizer_->optimize(iterations_inner_loop);
        /* Debug trajs: optimize */
        temp_traj.clear();
        for (auto& pt : teb_->poses()) {
            temp_traj.emplace_back(pt->pose().cast<float>());
        }
        debug_trajs_.emplace_back(temp_traj);
        if (!iter) {
            cout << "graphOptimize optimize failed! iter:" << iter << endl;
//            ALOGD("graphOptimize optimize failed! iter: %d", iter);
            clearGraph();
            return false;
        }

        computeCurrentCost();

        clearGraph();
    }
    return true;
}

bool PathOptimizer::buildGraph() {
    /* 1.Graph empty check */
    if (!optimizer_->edges().empty() || !optimizer_->vertices().empty()) {
        cout << "Graph optimizer_ is not empty!" << endl;
//        ALOGD("Graph optimizer_ is not empty!");
        return false;
    }

    /* 2.Add Vertices */
    addVertices();

    /* 3.Add Edges */
    addEdgesSlip();
    addEdgesVelocity();
    addEdgesAcceleration();
    addEdgesTimeOptimal();
    addEdgesKinematics();

    return true;
}

void PathOptimizer::addVertices() {
    // used for vertices ids
    int id_counter = 0;

    for (int i = 0; i < teb_->poses().size(); ++i) {
        teb_->poseVertex(i)->setId(id_counter++);
        optimizer_->addVertex(teb_->poseVertex(i));
        if (i < teb_->timeDiffs().size()) {
            teb_->timeDiffVertex(i)->setId(id_counter++);
            optimizer_->addVertex(teb_->timeDiffVertex(i));
        }
    }
}

void PathOptimizer::addEdgesSlip() {
    Eigen::Matrix<double, 1, 1> inform;
    inform.fill(cfg_->optimize.weight_slip);
    int pose_size = teb_->poses().size();
    computeSlopeAngles(q_, params_.g, cur_pose_[2], theta_slope_, psi_s_);
    for (int i = 0; i < pose_size - 1; ++i) {
        auto edge_slip = new EdgeSlip();
        edge_slip->setVertex(0, teb_->poseVertex(i));
        edge_slip->setVertex(1, teb_->poseVertex(i + 1));
        edge_slip->setInformation(inform);
        edge_slip->setSlopeInfo(theta_slope_, psi_s_, params_);
        optimizer_->addEdge(edge_slip);
    }
}

void PathOptimizer::addEdgesVelocity() {
    // if weight equals zero skip adding edges!
    if (cfg_->optimize.weight_max_lin_vel_forward < 1e-4
        && cfg_->optimize.weight_max_rot_vel < 1e-4) {
        return;
    }

    Eigen::Matrix<double, 2, 2> inform;
    inform << cfg_->optimize.weight_max_lin_vel_forward, 0.0f, 0.0f,
        cfg_->optimize.weight_max_rot_vel;

    int pose_size = teb_->poses().size();
    for (int i = 0; i < pose_size - 1; ++i) {
        auto edge_vel = new EdgeVelocity();
        edge_vel->setVertex(0, teb_->poseVertex(i));
        edge_vel->setVertex(1, teb_->poseVertex(i + 1));
        edge_vel->setVertex(2, teb_->timeDiffVertex(i));
        edge_vel->setInformation(inform);
        edge_vel->setParams(cfg_);

        optimizer_->addEdge(edge_vel);
    }
}

void PathOptimizer::addEdgesAcceleration() {
    // if weight equals zero skip adding edges!
    if (cfg_->optimize.weight_max_lin_acc < 1e-4 && cfg_->optimize.weight_max_rot_acc < 1e-4) {
        return;
    }

    Eigen::Matrix<double, 2, 2> inform;
    inform << cfg_->optimize.weight_max_lin_acc, 0.0f, 0.0f, cfg_->optimize.weight_max_rot_acc;

    int poses_size = teb_->poses().size();

    /* Start */
    auto edge_acc_start = new EdgeAccelerationStart();
    edge_acc_start->setVertex(0, teb_->poseVertex(0));
    edge_acc_start->setVertex(1, teb_->poseVertex(1));
    edge_acc_start->setVertex(2, teb_->timeDiffVertex(0));
    edge_acc_start->setInitialVelocity(vel_start_);
    edge_acc_start->setInformation(inform);
    edge_acc_start->setParams(cfg_);
    optimizer_->addEdge(edge_acc_start);

    /* Intermediate */
    for (int i = 0; i < poses_size - 2; ++i) {
        auto edge_acc = new EdgeAcceleration();
        edge_acc->setVertex(0, teb_->poseVertex(i));
        edge_acc->setVertex(1, teb_->poseVertex(i + 1));
        edge_acc->setVertex(2, teb_->poseVertex(i + 2));
        edge_acc->setVertex(3, teb_->timeDiffVertex(i));
        edge_acc->setVertex(4, teb_->timeDiffVertex(i + 1));
        edge_acc->setInformation(inform);
        edge_acc->setParams(cfg_);
        optimizer_->addEdge(edge_acc);
    }

    /* Goal */
    auto edge_acc_goal = new EdgeAccelerationGoal();
    edge_acc_goal->setVertex(0, teb_->poseVertex(poses_size - 2));
    edge_acc_goal->setVertex(1, teb_->poseVertex(poses_size - 1));
    edge_acc_goal->setVertex(2, teb_->timeDiffs().back());
    if (fix_final_goal_) {
        edge_acc_goal->setGoalVelocity(vel_goal_);
    } else {
        edge_acc_goal->setGoalVelocity(Eigen::Vector2f(cfg_->robot.max_lin_vel_forward, 0));
    }

    edge_acc_goal->setInformation(inform);
    edge_acc_goal->setParams(cfg_);
    optimizer_->addEdge(edge_acc_goal);
}

void PathOptimizer::addEdgesTimeOptimal() {
    // if weight equals zero skip adding edges!
    if (cfg_->optimize.weight_optimal_time < 1e-4) {
        return;
    }

    Eigen::Matrix<double, 1, 1> inform;
    inform.fill(cfg_->optimize.weight_optimal_time);

    for (auto& time_diff : teb_->timeDiffs()) {
        auto edge_time_optimal = new EdgeTimeOptimal();
        edge_time_optimal->setVertex(0, time_diff);
        edge_time_optimal->setInformation(inform);
        edge_time_optimal->setParams(cfg_);
        optimizer_->addEdge(edge_time_optimal);
    }
}

void PathOptimizer::addEdgesKinematics() {
    // if weight equals zero skip adding edges!
    if (cfg_->optimize.weight_kinematics_nh < 1e-4
        && cfg_->optimize.weight_kinematics_diff_drive < 1e-4) {
        return;
    }

    Eigen::Matrix<double, 2, 2> inform;
    inform << cfg_->optimize.weight_kinematics_nh, 0.0f, 0.0f,
        cfg_->optimize.weight_kinematics_diff_drive;

    for (int i = 0; i < teb_->poses().size() - 1; ++i) {
        auto edge_kinematics = new EdgeKinematics();
        edge_kinematics->setVertex(0, teb_->poseVertex(i));
        edge_kinematics->setVertex(1, teb_->poseVertex(i + 1));
        edge_kinematics->setInformation(inform);
        edge_kinematics->setParams(cfg_);
        optimizer_->addEdge(edge_kinematics);
    }
}

double PathOptimizer::computeCurrentCost() {
    error_poses_.clear();
    double cost = 0.0;
    if (optimizer_->edges().empty() && optimizer_->vertices().empty()) {
        return cost;
    }

    optimizer_->computeInitialGuess();

    //    double total_error = 0.0f, total_obs_error = 0.0f;
    //    for (const auto &edge:optimizer_->activeEdges()) {
    //        auto obs_edge = dynamic_cast<EdgeObstacle*>(edge);
    //        if (obs_edge != nullptr &&
    //            edge->errorData()[0] > 1e-2) {
    //            obs_edge->printPose();
    //            cout << "  --  ";
    //            total_obs_error += obs_edge->errorData()[0];
    //        }
    //        total_error += edge->errorData()[0];
    //    }
    //    cout << " Total error:" << total_error << endl;

    //    double total_error = 0.0f, total_obs_error = 0.0f, total_vel_error = 0.0f, total_acc_error = 0.0f, total_kinematic_error = 0.0f;

    double max_cost = 0.0;
    //    g2o::OptimizableGraph::Edge* max_cost_edge;
    for (const auto& edge : optimizer_->activeEdges()) {
        double cur_cost = edge->chi2();

        cost += cur_cost;

        if (cur_cost > max_cost) {
            max_cost = cur_cost;
            //            max_cost_edge = edge;
        }
        //        if (cur_cost > 1e-2) {
        //            if (dynamic_cast<EdgeObstacle *>(edge) != nullptr) {
        //                dynamic_cast<EdgeObstacle *>(edge)->printPose();
        //                error_poses_.emplace_back(dynamic_cast<EdgeObstacle *>(edge)->getPose());
        //                cout << " EdgeObstacle cost:";
        //            } else if (dynamic_cast<EdgeVelocity *>(edge) != nullptr) {
        //                cout << "EdgeVelocity cost:";
        //            } else if (dynamic_cast<EdgeTimeOptimal *>(edge) != nullptr) {
        //                cout << "EdgeTimeOptimal cost:";
        //            } else if (dynamic_cast<EdgeKinematics *>(edge) != nullptr) {
        //                cout << "EdgeKinematics cost:";
        //            } else if (dynamic_cast<EdgeAcceleration *>(edge) != nullptr) {
        //                cout << "EdgeAcceleration cost:";
        //            } else if (dynamic_cast<EdgeAccelerationStart *>(edge) != nullptr) {
        //                cout << "EdgeAccelerationStart cost:";
        //            } else if (dynamic_cast<EdgeAccelerationGoal *>(edge) != nullptr) {
        //                cout << "EdgeAccelerationGoal cost:";
        //            }
        //            cout << cur_cost << endl;
        //        }
    }

    //    if (dynamic_cast<EdgeObstacle *>(max_cost_edge) != nullptr) {
    //        cout << "EdgeObstacle cost:";
    //    } else if (dynamic_cast<EdgeVelocity *>(max_cost_edge) != nullptr) {
    //        cout << "EdgeVelocity cost:";
    //    } else if (dynamic_cast<EdgeTimeOptimal *>(max_cost_edge) != nullptr) {
    //        cout << "EdgeTimeOptimal cost:";
    //    } else if (dynamic_cast<EdgeKinematics *>(max_cost_edge) != nullptr) {
    //        cout << "EdgeKinematics cost:";
    //    } else if (dynamic_cast<EdgeAcceleration *>(max_cost_edge) != nullptr) {
    //        cout << "EdgeAcceleration cost:";
    //    } else if (dynamic_cast<EdgeAccelerationStart *>(max_cost_edge) != nullptr) {
    //        cout << "EdgeAccelerationStart cost:";
    //    } else if (dynamic_cast<EdgeAccelerationGoal *>(max_cost_edge) != nullptr) {
    //        cout << "EdgeAccelerationGoal cost:";
    //    }
    //    cout << max_cost << " computeCurrentCost :" << cost << endl;
    return cost;
}

void PathOptimizer::init(const ros::NodeHandle& nh) {
    node_ = nh;
    clearGraph();
    teb_->clearVertexSequences();
    params_.mass = 20.0f;
    params_.cgHeight = 0.25;
    params_.g = 9.81;
    params_.rearWeightFraction = 0.7;
    params_.trackWidth = 0.48;
    params_.wheelbase = 0.45;
    teb_debug_pub_ = node_.advertise<visualization_msgs::MarkerArray>("/teb/debug_path", 1);
    teb_result_pub_ = node_.advertise<nav_msgs::Path>("/teb/result", 1);
    std::cout << "debug path optimizer init finish" << std::endl;
}

void PathOptimizer::clearGraph() {
    if (optimizer_) {
        optimizer_->vertices().clear();
        optimizer_->clear();
    }
}

bool PathOptimizer::calcCmd() {
    // todo: lookahead affect motion smooth?
    //    int lookahead = std::min<int>(teb_->timeDiffs().size(), 3);
    int lookahead = std::min<int>(teb_->timeDiffs().size(), 2);

    float dt = 0.0f;
    for (int i = 0; i < lookahead; ++i) {
        dt += (float)teb_->timeDiff(i);
    }
    if (dt < 1e-3) {
        return false;
    }
    cmd_ = extractVelocity(teb_->pose(0), teb_->pose(lookahead), dt);

    //    if (fabs(cmd_[0]) < 1e-1 &&
    //        fabs(cmd_[1]) < 1e-1) {
    //        cout << "calcCmd too small dt:" << dt << " pose:("
    //             << teb_->pose(0)[0] << ", "
    //             << teb_->pose(0)[1] << ", "
    //             << teb_->pose(0)[2] << ") ("
    //             << teb_->pose(lookahead)[0] << ", "
    //             << teb_->pose(lookahead)[1] << ", "
    //             << teb_->pose(lookahead)[2] << ")"
    //             << endl;
    //    }

    /* Range limit */
    // Pure rotate
    //    if (fabs(cmd_[1]) < 1e-2)
    //        if (cmd_[0] > cfg_->robot.max_lin_vel_forward) {
    //
    //        } else if (cmd_[0] < -cfg_->robot.max_lin_vel_backward) {
    //
    //        }
    //    }
    //    if (cmd_[0] < -cfg_->robot.max_lin_vel_backward ||
    //        cmd_[0] > cfg_->robot.max_lin_vel_forward ||
    //        fabs(cmd_[1]) > cfg_->robot.max_rot_vel) {
    //        return false;
    //    }

    return true;
}

Eigen::Vector2f PathOptimizer::extractVelocity(const Eigen::Vector3d& pose_1,
                                               const Eigen::Vector3d& pose_2, double dt) {
    Eigen::Vector2f cmd(0.0f, 0.0f);
    if (dt < 1e-4) {
        return cmd;
    }

    Eigen::Vector3d pose_diff = pose_2 - pose_1;
    Eigen::Vector2d conf1dir(cos(pose_1[2]), sin(pose_1[2]));
    float dir = pose_diff.head(2).dot(conf1dir);
    if (dir > 0.0f) {
        cmd[0] = pose_diff.head(2).norm() / dt;
    } else {
        cmd[0] = -pose_diff.head(2).norm() / dt;
    }
    cmd[1] = uneven_planner::wrapToPi(pose_diff[2]) / dt;

    return cmd;
}

float PathOptimizer::getSumOfAllTimeDiffs() {
    return teb_->getSumOfAllTimeDiffs();
}

std::vector<Eigen::Vector3f> PathOptimizer::getInitPath() {
    if (ref_traj_.empty()) {
        return {};
    }
    std::vector<Eigen::Vector3f> init_path;
    init_path.emplace_back(ref_traj_.at(0));
    float dist = 0.0f;
    for (size_t i = 1; i < ref_traj_.size(); i++) {
        dist += (ref_traj_.at(i) - init_path.back()).head(2).norm();
        init_path.emplace_back(ref_traj_.at(i));
        if (dist > PathLength) {
            break;
        }
    }
    if (init_path.size() < 2) {
        init_path.insert(init_path.begin(), cur_pose_);
    }
//    ALOGD("get init path size: %d, total size: %d", init_path.size(), ref_traj_.size());
    return init_path;
}

std::vector<Eigen::Vector3f> PathOptimizer::getNextBackPath() {
    std::vector<Eigen::Vector3f> next_path;
    float remain_dist = 0;
    for (size_t i = 1; i < best_traj_.size(); i++) {
        remain_dist += (best_traj_.at(i) - best_traj_.at(i - 1)).head(2).norm();
    }
//    ALOGD("debug teb remain dist: %f", remain_dist);
    if (remain_dist < PathLength) {
        next_nearest_index_ =
            getNearestWayPoint(ref_traj_, best_traj_.back(), nearest_idx_, goal_index_, 10e6);
        remain_dist += (ref_traj_.at(next_nearest_index_) - best_traj_.back()).head(2).norm();
        next_path.emplace_back(ref_traj_.at(next_nearest_index_));
//        ALOGD(
//            "debug last nearest index: %d, remain dist: %f, cur nearest index: %d, goal index: %d",
//            next_nearest_index_, remain_dist, nearest_idx_, goal_index_);
        for (size_t i = next_nearest_index_ + 1; i < ref_traj_.size(); i++) {
            remain_dist += (ref_traj_.at(i) - next_path.back()).head(2).norm();
//            ALOGD("debug current index: %d, remain dist: %f", i, remain_dist);
            next_path.emplace_back(ref_traj_.at(i));
            if (remain_dist > PathLength) {
//                ALOGD("path update to index: %d", i);
                break;
            }
        }
    }
    return next_path;
}