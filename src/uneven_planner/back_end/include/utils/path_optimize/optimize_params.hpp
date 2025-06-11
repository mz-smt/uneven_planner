//
// Created by evanlin on 5/28/20.
//

#ifndef MOTIONPLANNER_OPTIMIZEPARAMS_HPP
#define MOTIONPLANNER_OPTIMIZEPARAMS_HPP

namespace ninebot_algo {
    namespace motion_planner {

        class OptimizeParams {
        public:
            struct Trajectory {
            } trajectory{};

            struct Robot {
                float max_lin_vel_forward;
                float max_lin_vel_backward;
                float max_lin_acc;

                float max_rot_vel;
                float max_rot_acc;
            } robot{};

            struct Obstacles {
                float min_obs_dist;
            } obstacles{};

            struct GoalTolerance {
                float xy_goal_tolerance;
                float yaw_goal_tolerance;
            } goal_tolerance{};

            struct Optimization {
                /* TEB time step */
                float time_step;

                /* Iteration num */
                int num_outer_iterations;
                // Inside g2o optimizer
                int num_inner_iterations;

                /* Edges weight */
                float weight_max_lin_vel_forward;
                float weight_max_lin_acc;

                float weight_max_rot_vel;
                float weight_max_rot_acc;

                float weight_kinematics_nh;
                float weight_kinematics_diff_drive;

                float weight_optimal_time;
                float weight_shortest_path;
                float weight_obstacle;
                float weight_slip;
            } optimize{};

            OptimizeParams() {
                /* Robot */
                //                robot.max_lin_vel_forward = 0.4f;
                //                robot.max_lin_vel_backward = 0.2f;
                //                robot.max_rot_vel = 0.3f;
                //
                //                robot.max_lin_acc = 0.5;
                //                robot.max_rot_acc = 0.5;

                robot.max_lin_vel_forward = 0.5f;
                robot.max_lin_vel_backward = 0.2f;
                robot.max_rot_vel = 0.5f;

                robot.max_lin_acc = 0.5;  // 0.04 * 20
                robot.max_rot_acc = 0.5;  // 0.1 * 20

                /* Obstacle */
                obstacles.min_obs_dist = 0.8f;

                /* GoalTolerance */
                goal_tolerance.xy_goal_tolerance = 0.2f;
                goal_tolerance.yaw_goal_tolerance = 0.174f;

                /* Optimize */
                optimize.time_step = 0.3f;

                optimize.num_inner_iterations = 10;
                optimize.num_outer_iterations = 100;

                optimize.weight_max_lin_vel_forward = 2;
                optimize.weight_max_rot_vel = 1;

                optimize.weight_max_lin_acc = 10;
                optimize.weight_max_rot_acc = 10;
                //                optimize.weight_max_lin_acc = 1000;
                //                optimize.weight_max_rot_acc = 1000;

                optimize.weight_kinematics_nh = 1000;
                optimize.weight_kinematics_diff_drive = 1;

                optimize.weight_optimal_time = 0.1;
                optimize.weight_shortest_path = 0;
                optimize.weight_obstacle = 100;
                optimize.weight_slip = 10;
            }
        };
    }  // namespace motion_planner
}  // namespace ninebot_algo

#endif  //MOTIONPLANNER_OPTIMIZEPARAMS_HPP
