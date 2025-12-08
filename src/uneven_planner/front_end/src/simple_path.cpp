//
// Created by mazheng on 25-5-8.
//

#include <tf/transform_datatypes.h>
#include "front_end/simple_path.hpp"
#include "utils/math_util.hpp"
#include "nav_msgs/Path.h"

using namespace std;
using namespace uneven_planner;

SimplePath::SimplePath(float rotate_step_, float move_step_, bool force_forward_)
        : ROTATE_STEP(rotate_step_), MOVE_STEP(move_step_), force_forward(force_forward_) {}

void SimplePath::init(ros::NodeHandle &nh) {
    simple_path_pub_ = nh.advertise<nav_msgs::Path>("/simple_path/result_path", 1);
}

void SimplePath::setSampleParam(float rotate_step, float move_step) {
    ROTATE_STEP = rotate_step;
    MOVE_STEP = move_step;
}

bool SimplePath::generatePath(const Eigen::Vector3f& start_pt, const Eigen::Vector3f& end_pt,
                              vector<Eigen::Vector3f>& optimal_path) {
    optimal_path.clear();
    Eigen::Vector3f pose_diff = end_pt - start_pt;
    float connection_angle = atan2(pose_diff.y(), pose_diff.x());
    float connection_line_rads;
    float start_agl = wrapToPi(start_pt[2]);

    /* 1.Calc connection angle */
    if (force_forward || theta_free) {
        connection_line_rads = connection_angle;
        direction = true;
    } else {
        float rotate_sum = abs(wrapToPi(connection_angle - start_agl))
                           + abs(wrapToPi(end_pt[2] - connection_angle));

        if (rotate_sum < M_PI) {
            // Forward
            connection_line_rads = connection_angle;
            direction = true;
        } else {
            // Backward
            connection_line_rads = wrapToPi(connection_angle + (float)M_PI);
            direction = false;
        }
    }

    if (theta_free) {
        steer = abs(wrapToPi(connection_line_rads - start_agl));
    } else {
        steer = abs(wrapToPi(connection_line_rads - start_agl))
                + abs(wrapToPi(end_pt[2] - connection_angle));
    }

    /* 2.Calc key points */
    Eigen::Vector3f first_turn_pt = start_pt;
    Eigen::Vector3f move_pt = end_pt;

    /* 2.1 First turn */
    first_turn_pt[2] = connection_line_rads;

    /* 2.2 Move */
    move_pt[2] = connection_line_rads;

    //    cout << "first_turn_pt:(" <<first_turn_pt[0]
    //         << ", " << first_turn_pt[1]
    //         << ", " << first_turn_pt[2]
    //         << ") move_pt:(" << move_pt[0]
    //         << ", " << move_pt[1]
    //         << ", " << move_pt[2] << ")" << endl;

    /* 2.3 Second turn */
    // Target is end_pt

    /* 3.Interpolate */
    bool need_reverse = false;
    if (set_forbid_dir_) {
        need_reverse = crossesForbidden(start_agl, first_turn_pt[2], forbidden_dir_);

    }
    int rotate_cnt = static_cast<int>(trunc(wrapToPi(first_turn_pt[2] - start_agl) / ROTATE_STEP));
    //    cout << "rotate_cnt:" << wrapToPi(first_turn_pt[2] - start_agl) << " cnt:" << rotate_cnt << endl;
    float rotate_step = rotate_cnt > 0 ? ROTATE_STEP : -ROTATE_STEP;
    rotate_cnt = abs(rotate_cnt);
    if (need_reverse) {
        auto angle_diff = wrapToPi(first_turn_pt[2] - start_agl);
        auto reverse_angle = angle_diff;
        if (angle_diff > 0) {
            reverse_angle = angle_diff - 2 * M_PI;
        } else {
            reverse_angle = angle_diff + 2 * M_PI;
        }
        rotate_cnt = static_cast<int>(trunc(reverse_angle / ROTATE_STEP));
        rotate_step = rotate_cnt > 0 ? ROTATE_STEP : -ROTATE_STEP;
        rotate_cnt = abs(rotate_cnt);

    }
    optimal_path.emplace_back(start_pt[0], start_pt[1], wrapToPi(start_agl));
    for (int i = 1; i <= rotate_cnt; ++i) {
        optimal_path.emplace_back(start_pt[0], start_pt[1], wrapToPi(start_agl + rotate_step * i));
    }

    int move_cnt = static_cast<int>(trunc(pose_diff.head(2).norm() / MOVE_STEP));
    Eigen::Vector2f move_delta = pose_diff.head(2) / move_cnt;
    optimal_path.emplace_back(first_turn_pt[0], first_turn_pt[1], wrapToPi(first_turn_pt[2]));
    for (int j = 1; j <= move_cnt; ++j) {
        optimal_path.emplace_back(first_turn_pt[0] + move_delta[0] * j,
                                  first_turn_pt[1] + move_delta[1] * j, wrapToPi(first_turn_pt[2]));
    }

    if (!theta_free) {
        optimal_path.emplace_back(move_pt[0], move_pt[1], wrapToPi(move_pt[2]));
        need_reverse = false;
        if (set_forbid_dir_) {
            need_reverse = crossesForbidden(move_pt[2], end_pt[2], forbidden_dir_);

        }
        rotate_cnt = static_cast<int>(trunc(wrapToPi(end_pt[2] - move_pt[2]) / ROTATE_STEP));
        //    cout << "move_pt:(" << move_pt[0] << ", " << move_pt[1] << ", " << move_pt[2] << ") "
        //         << "end_pt:(" << end_pt[0] << ", " << end_pt[1] << ", " << end_pt[2] << ") "
        //         << "rotate_cnt:" << wrapToPi(end_pt[2] - move_pt[2]) << " cnt:" << rotate_cnt << endl;
        rotate_step = rotate_cnt > 0 ? ROTATE_STEP : -ROTATE_STEP;
        rotate_cnt = abs(rotate_cnt);
        if (need_reverse) {
            auto angle_diff = wrapToPi(end_pt[2] - move_pt[2]);
            auto reverse_angle = angle_diff;
            if (angle_diff > 0) {
                reverse_angle = angle_diff - 2 * M_PI;
            } else {
                reverse_angle = angle_diff + 2 * M_PI;
            }
            rotate_cnt = static_cast<int>(trunc(reverse_angle / ROTATE_STEP));
            rotate_step = rotate_cnt > 0 ? ROTATE_STEP : -ROTATE_STEP;
            rotate_cnt = abs(rotate_cnt);

        }
        for (int k = 1; k <= rotate_cnt; ++k) {
            optimal_path.emplace_back(move_pt[0], move_pt[1],
                                      wrapToPi(move_pt[2] + rotate_step * k));
        }
        optimal_path.emplace_back(end_pt);
    } else {
        optimal_path.emplace_back(move_pt[0], move_pt[1], wrapToPi(move_pt[2]));
    }
    optimal_path.emplace_back(bodyFrameToGroundFrame(Eigen::Vector3f(0.5, 0.0, 0.0), optimal_path.back()));
//    Eigen::Vector3f  forward_pt{0.2, 0, 0.0};
//    optimal_path.emplace_back(bodyFrameToGroundFrame(forward_pt, optimal_path.back()));
    nav_msgs::Path simple_path;
    simple_path.header.stamp = ros::Time::now();
    simple_path.header.frame_id = "world";
    geometry_msgs::PoseStamped temp_pose;
    temp_pose.header = simple_path.header;
    simple_path.poses.clear();
    for (const auto& point : optimal_path) {
        temp_pose.pose.position.x = point[0];
        temp_pose.pose.position.y = point[1];
        temp_pose.pose.orientation = tf::createQuaternionMsgFromYaw(point[2]);
        simple_path.poses.push_back(temp_pose);
    }
    simple_path_pub_.publish(simple_path);

    return true;
}