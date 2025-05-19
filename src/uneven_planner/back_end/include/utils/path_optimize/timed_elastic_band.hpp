//
// Created by evanlin on 5/29/20.
//

#ifndef MOTIONPLANNER_TIMEDELASTICBAND_HPP
#define MOTIONPLANNER_TIMEDELASTICBAND_HPP

#include <iostream>
#include <vector>

#include "vertex_pose.hpp"
#include "vertex_time_diff.hpp"

namespace ninebot_algo {
    namespace motion_planner {

        using PoseSequence = std::vector<VertexPose*>;
        using TimeDiffSequence = std::vector<VertexTimeDiff*>;

        class TimedElasticBand {
            friend class TimedElasticBandTest;

        public:
            TimedElasticBand() = default;

            virtual ~TimedElasticBand() = default;

            /* -----> Init path to vertex sequences <----- */
            bool initBandSequence(const std::vector<Eigen::Vector3f>& path,
                                  const Eigen::Vector3f cur_pose, float max_lin_vel,
                                  float max_rot_vel);

            bool addPosesAndTimeDiffs(const std::vector<Eigen::Vector3f>& path);

            /* Is init */
            bool isInit() {
                return !pose_vec_.empty() && !time_diff_vec_.empty();
            }

            float poseDistance(const Eigen::Vector3f& pose_a, const Eigen::Vector3f& pose_b);

            void updateAndPruneTEB(const Eigen::Vector3f& pose, int min_samples = 3);

            /* Path resize */
            void autoResize(float time_step);

            /* Get Sum of All TimeDiffs */
            float getSumOfAllTimeDiffs();

            /* -----> Clear vertex sequences <----- */
            void clearVertexSequences();

            /* -----> Access pose and time_diff sequences <----- */
            /* VertexPose API */
            const PoseSequence& poses() const {
                return pose_vec_;
            }

            const Eigen::Vector3d& pose(int index) const {
                assert(index >= 0 && index < pose_vec_.size());
                return pose_vec_.at(index)->pose();
            }

            const Eigen::Vector3d& backPose() const {
                assert(!pose_vec_.empty());
                return pose_vec_.back()->pose();
            }

            VertexPose* poseVertex(int index) {
                assert(index >= 0 && index < pose_vec_.size());
                return pose_vec_.at(index);
            }

            /* VertexTimeDiff API */
            const TimeDiffSequence& timeDiffs() const {
                return time_diff_vec_;
            }

            const double& timeDiff(int index) const {
                assert(index >= 0 && index < time_diff_vec_.size());
                return time_diff_vec_.at(index)->dt();
            }

            const double& backTimeDiff() const {
                assert(!time_diff_vec_.empty());
                return time_diff_vec_.back()->dt();
            }

            VertexTimeDiff* timeDiffVertex(int index) {
                assert(index >= 0 && index < time_diff_vec_.size());
                return time_diff_vec_.at(index);
            }

            void setBackPoseFixed() {
                pose_vec_.back()->setFixed(true);
            }

        private:
            /* Insert and delete */
            void insertPose(int index, const Eigen::Vector3d& pose) {
                auto pose_vertex = new VertexPose(pose);
                pose_vec_.insert(pose_vec_.begin() + index, pose_vertex);
            }

            void deletePose(int index) {
                assert(index < pose_vec_.size());
                delete pose_vec_.at(index);
                pose_vec_.erase(pose_vec_.begin() + index);
            }

            void deletePose(int index, int number) {
                assert(index + number <= pose_vec_.size());
                for (int i = index; i < index + number; ++i) {
                    delete pose_vec_.at(i);
                }
                pose_vec_.erase(pose_vec_.begin() + index, pose_vec_.begin() + index + number);
            }

            void insertTimeDiff(int index, double dt) {
                auto time_diff_vertex = new VertexTimeDiff(dt);
                time_diff_vec_.insert(time_diff_vec_.begin() + index, time_diff_vertex);
            }

            void deleteTimeDiff(int index) {
                assert(index < time_diff_vec_.size());
                delete time_diff_vec_.at(index);
                time_diff_vec_.erase(time_diff_vec_.begin() + index);
            }

            void deleteTimeDiff(int index, int number) {
                assert(index + number <= time_diff_vec_.size());
                for (int i = index; i < index + number; ++i) {
                    delete time_diff_vec_.at(i);
                }
                time_diff_vec_.erase(time_diff_vec_.begin() + index,
                                     time_diff_vec_.begin() + index + number);
            }
            float calcTimeDiff(Eigen::Vector3d start_pt, Eigen::Vector3d end_pt);

            void adjustRotatePoints();

            void initPrune(Eigen::Vector3f cur_pose);

        protected:
            PoseSequence pose_vec_;
            TimeDiffSequence time_diff_vec_;
            float max_lin_vel_;
            float max_rot_vel_;

        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        };
    }  // namespace motion_planner
}  // namespace ninebot_algo

#endif  //MOTIONPLANNER_TIMEDELASTICBAND_HPP
