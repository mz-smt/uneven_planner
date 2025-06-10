//
// Created by evanlin on 5/28/20.
//

#ifndef MOTIONPLANNER_PATHOPTIMIZER_HPP
#define MOTIONPLANNER_PATHOPTIMIZER_HPP

#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>

#include <Eigen/Dense>
#include <memory>
#include <ros/ros.h>
#include <std_msgs/ColorRGBA.h>
#include "utils/math_util.hpp"

/* Debug info */
//#define DEBUG_OPTIMIZER
#define DEBUG_TRAJS

/* Prune type */
#define DIRECTLY_PRUNE 1
#define FINALLY_PRUNE 2
#define NOT_UPDATE_PRUNE 3

#define PRUNE_TYPE DIRECTLY_PRUNE
//#define PRUNE_TYPE FINALLY_PRUNE

/* Time profiler */
#define TIME_PROFILE

namespace TestMissionPlanner {
    class MissionOpenSpaceAgent;
}

namespace TestRecoveryMission {
    class MissionAgent;
}

namespace ninebot_algo {
    namespace motion_planner {

        enum class PlannerStatus {
            Error = -1,
            Normal = 0,
            Goal_Reached = 1,
            Failed = 2
        };

        /* Using type */
        using BlockSolver = g2o::BlockSolver<g2o::BlockSolverTraits<-1, -1>>;
        //        using LinearSolver = g2o::LinearSolverCSparse<BlockSolver::PoseMatrixType>;
        using LinearSolver = g2o::LinearSolverEigen<BlockSolver::PoseMatrixType>;
        using ViaPointContainer =
            std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>>;

        /* Forward declaration */
        class OptimizeParams;

        class TimedElasticBand;

        class BlurDistanceMap;

        class DistanceMapInterface;

        /* Class PathOptimizer */
        class PathOptimizer {

        public:
            explicit PathOptimizer();

            ~PathOptimizer();
            /**
             * @brief Ground
             * @param traj
             */
            void  setReferenceTrajectory(const std::vector<Eigen::Vector3f>& traj);

            PlannerStatus makePlan(const Eigen::Vector3f& pose, const Eigen::Vector2f& speed);

            /**
             * @brief Ground
             * @return
             */
            std::vector<Eigen::Vector3f> getOptimalTrajectory();

            Eigen::Vector2f getOptimalControlCmd();

            /**
             * @brief Ground
             * @return
             */
            Eigen::Vector3f getCurrentTarget();

            void setGoalVel(Eigen::Vector2f vel_goal);

            std::vector<Eigen::Vector3f> getVelocityProfile() const;

            // todo: feasible check, intermediate pose collison check
            //            isTrajectoryFeasible
            /* get sum of all time_diff */
            float getSumOfAllTimeDiffs();

            void init(const ros::NodeHandle& nh);

            void setQuaternion(const Eigen::Quaterniond& q) {
                q_ = q;
            }

        private:
            void registerG2OTypes();

            bool graphOptimize(int iterations_inner_loop, int iterations_outer_loop);

            /* -----> Build graph <----- */
            bool buildGraph();

            void addVertices();

            void addEdgesSlip();

            void addEdgesVelocity();

            void addEdgesAcceleration();

            void addEdgesTimeOptimal();

            void addEdgesKinematics();

            /* -----> Calc cost <----- */
            double computeCurrentCost();

            /* -----> Build graph <----- */

            void clearGraph();

            bool calcCmd();

            std::vector<Eigen::Vector3f> getInitPath();

            std::vector<Eigen::Vector3f> getNextBackPath();

            static void computeSlopeAngles(const Eigen::Quaterniond& q, double g, double yaw,
                                           float& theta_slope, float& psi_s) {
                // 1. 构造机体->世界旋转矩阵（四元数转旋转矩阵）
                Eigen::Matrix3d Rwb = q.toRotationMatrix();  // :contentReference[oaicite:4]{index=4}

                // 2. 世界重力 [0,0,-g] 投影到机体坐标系
                Eigen::Vector3d g_body = Rwb.transpose() * Eigen::Vector3d(0.0, 0.0, -g);  //

                // 分量
                double gx = g_body.x();
                double gy = g_body.y();
                double gz = g_body.z();

                // 3. 计算坡度角
                theta_slope = std::atan2(std::hypot(gx, gy), std::abs(gz));  //

                // 4. 机体坐标系下坡面上升方向
                double psi_body = std::atan2(-gy, -gx);  // :contentReference[oaicite:5]{index=5}

                // 5. 规范化输入 yaw 到 (-π, π]
                double yaw_n =
                        std::atan2(std::sin(yaw), std::cos(yaw));  // :contentReference[oaicite:6]{index=6}

                // 6. 全局系下坡度方向角 = yaw_n + psi_body，再规范化
                double raw = yaw_n + psi_body;
                psi_s =
                        std::atan2(std::sin(raw), std::cos(raw));  // :contentReference[oaicite:7]{index=7}
            }

            std_msgs::ColorRGBA hsvToRgba(float h, float s, float v, float a=1.0f) {
                float r, g, b;
                int i = int(h * 6);
                float f = h*6 - i, p = v*(1-s), q = v*(1-f*s), t = v*(1-(1-f)*s);
                switch(i % 6) {
                    case 0: r=v; g=t; b=p; break;
                    case 1: r=q; g=v; b=p; break;
                    case 2: r=p; g=v; b=t; break;
                    case 3: r=p; g=q; b=v; break;
                    case 4: r=t; g=p; b=v; break;
                    case 5: r=v; g=p; b=q; break;
                }
                std_msgs::ColorRGBA c; c.r=r; c.g=g; c.b=b; c.a=a; return c;
            }

        public:
            static Eigen::Vector2f extractVelocity(const Eigen::Vector3d& pose_1,
                                                   const Eigen::Vector3d& pose_2, double dt);

        private:
            std::shared_ptr<g2o::SparseOptimizer> optimizer_;

            std::shared_ptr<OptimizeParams> cfg_;
            std::unique_ptr<TimedElasticBand> teb_;

            Eigen::Vector2f vel_start_;
            Eigen::Vector2f vel_goal_;
            Eigen::Vector3f cur_pose_;
            Eigen::Vector2f cmd_;

            /* Ground coordinate */
            std::vector<Eigen::Vector3f> best_traj_;

            /* Ref traj */
            std::vector<Eigen::Vector3f> ref_traj_;
            int goal_index_;
            int nearest_idx_;
            int next_nearest_index_;
            bool path_update_;
            bool fix_final_goal_;


            std::vector<Eigen::Vector3f> error_poses_;
            const float PathLength = 2.0f;
            ros::NodeHandle node_;
            Eigen::Quaterniond q_;
            float theta_slope_;
            float psi_s_;
            uneven_planner::VehicleParams params_;
            std::vector<std::vector<Eigen::Vector3d>> debug_trajs_;
            std::vector<std::vector<float>> traj_cost_vec_;
            ros::Publisher teb_debug_pub_;
            ros::Publisher teb_cost_pub_;
            ros::Publisher teb_result_pub_;
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        };
    }  // namespace motion_planner
}  // namespace ninebot_algo

#endif  //MOTIONPLANNER_PATHOPTIMIZER_HPP
