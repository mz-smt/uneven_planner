//
// Created by evanlin on 5/29/20.
//

#include "utils/path_optimize/timed_elastic_band.hpp"

#include <cmath>
#include <iomanip>

//#include "eigen_util/EigenUtil.hpp"
#include "utils/math_util.hpp"
//#include "ninebot_log.h"

using namespace std;
using namespace ninebot_algo;
using namespace ninebot_algo::motion_planner;

bool TimedElasticBand::initBandSequence(const std::vector<Eigen::Vector3f>& path,
                                        const Eigen::Vector3f cur_pose, float max_lin_vel,
                                        float max_rot_vel) {
    if (path.empty()) {
        return false;
    }
    clearVertexSequences();

    max_lin_vel_ = max_lin_vel;
    max_rot_vel_ = max_rot_vel;
#if 1
    /* Start pose vertex */
    pose_vec_.emplace_back(new VertexPose(path.front()));
    pose_vec_.at(0)->setFixed(true);
    //    cout << "Add TimedElasticBand start:" << 0
    //         << " pose:(" << path.front()[0]
    //         << ", " << path.front()[1]
    //         << ", " << path.front()[2]
    //         << ")" << endl;

    /* Add poses */
    for (int i = 1; i < path.size(); ++i) {
        double dt = calcTimeDiff(path[i].cast<double>(), pose_vec_.back()->pose());
        //        Eigen::Vector3d pose_diff = path[i].cast<double>() - pose_vec_.back()->pose();
        //
        //        double dt_rot = fabs(wrapToPi(pose_diff[2]) / max_rot_vel);
        //        double dt_lin = pose_diff.head(2).norm() / max_lin_vel;
        //        double dt = std::max(dt_rot, dt_lin);

        //        if (1 == i) {
        //            std::cout << std::fixed << std::setprecision(4)
        //                      << "pose_diff:("
        //                      << pose_diff[0] << ", "
        //                      << pose_diff[1] << ", "
        //                      << pose_diff[2] << ")"
        //                      << " dt_lin:" << dt_lin
        //                      << " dt_rot:" << dt_rot
        //                      << " dt:" << dt
        //                      << endl;
        //        }

        if (dt < 1e-4) {
            // Skip duplicate point
            //            cout << "Skip TimedElasticBand pose " << i
            //                 << ":(" << path[i][0]
            //                 << ", " << path[i][1]
            //                 << ", " << path[i][2]
            //                 << ") pose " << i - 1
            //                 << ":(" << pose_vec_.back()->pose()[0]
            //                 << ", " << pose_vec_.back()->pose()[1]
            //                 << ", " << pose_vec_.back()->pose()[2]
            //                 << ")  dt:" << dt
            //                 << " dt_rot:" << dt_rot
            //                 << " dt_lin:" << dt_lin << endl;
            continue;
        }

        pose_vec_.emplace_back(new VertexPose(path[i]));
        time_diff_vec_.emplace_back(new VertexTimeDiff(dt));
        //        cout << "Add TimedElasticBand pose " << i
        //             << ":(" << path[i][0]
        //             << ", " << path[i][1]
        //             << ", " << path[i][2]
        //             << ")  dt:" << dt << endl;
    }
    //	adjustRotatePoints();
    //	for (int i = 1; i < pose_vec_.size(); i++) {
    //		cout << "RotatePrune pose: " << i
    //             << ":(" << pose_vec_.at(i)->pose()[0]
    //             << ", " << pose_vec_.at(i)->pose()[1]
    //             << ", " << pose_vec_.at(i)->pose()[2]
    //             << ")  dt:" << time_diff_vec_.at(i - 1)->dt() << endl;
    //	}
    /* Goal pose vertex set fixed */
//    pose_vec_.back()->setFixed(true);
#else
    /* Start pose vertex */
    pose_vec_.emplace_back(new VertexPose(path.front()));
    pose_vec_.at(0)->setFixed(true);

    /* Add poses */
    for (int i = 1; i < path.size() - 1; ++i) {
        Eigen::Vector3f pose_diff = path[i + 1] - path[i];

        /* Remove duplicate point */
        if (fabs(pose_diff[0]) < 1e-3 && fabs(pose_diff[1]) < 1e-3 && fabs(pose_diff[2]) < 1e-3) {
            continue;
        }

        /* Average angle */
        //        float yaw = std::atan2(pose_diff.y(), pose_diff.x());
        float yaw =
            atan2((sin(path[i][2]) + sin(path[i + 1][2])), (cos(path[i][2]) + cos(path[i + 1][2])));

        double dt =
            (path[i].cast<double>() - pose_vec_.back()->pose()).head(2).norm() / max_lin_vel;

        /* Update to container */
        pose_vec_.emplace_back(new VertexPose(path[i][0], path[i][1], yaw));
        time_diff_vec_.emplace_back(new VertexTimeDiff(dt));

        //        cout << "Add TimedElasticBand pose " << i
        //             << ":(" << path[i][0]
        //             << ", " << path[i][1]
        //             << ", " << path[i][2]
        //             << ")  dt:" << dt
        //             << " yaw:" << yaw << endl;
    }
    /* Goal pose vertex set fixed */
    double dt =
        (path.back().cast<double>() - pose_vec_.back()->pose()).head(2).norm() / max_lin_vel;

    pose_vec_.emplace_back(new VertexPose(path.back()));
    pose_vec_.back()->setFixed(true);
    time_diff_vec_.emplace_back(new VertexTimeDiff(dt));

#endif  //0

    /* Prune for Hybrid A star result delay */
    initPrune(cur_pose);
    //    int cnt = 0;
    //    for (const auto &pose:pose_vec_) {
    //        cout << "pose_vec_:" << cnt
    //             << " " << *pose;
    //        if (cnt < time_diff_vec_.size()) {
    //            cout << "  time_diff_vec_:" << cnt
    //                 << " " << time_diff_vec_[cnt]->dt()
    //                 << endl;
    //        }
    //        cnt++;
    //    }
    //    cout << endl;

    //    this->autoResize(0.3f);

    return true;
}

bool TimedElasticBand::addPosesAndTimeDiffs(const std::vector<Eigen::Vector3f>& path) {
    /* Add poses */
    for (int i = 0; i < path.size(); ++i) {
        double dt = calcTimeDiff(path[i].cast<double>(), pose_vec_.back()->pose());
        if (dt < 1e-4) {
            continue;
        }
        pose_vec_.emplace_back(new VertexPose(path[i]));
        time_diff_vec_.emplace_back(new VertexTimeDiff(dt));
    }
    return true;
}

float TimedElasticBand::calcTimeDiff(Eigen::Vector3d start_pt, Eigen::Vector3d end_pt) {
    Eigen::Vector3d pose_diff = start_pt - end_pt;

    double dt_rot = fabs(uneven_planner::wrapToPi(pose_diff[2]) / max_rot_vel_);
    double dt_lin = pose_diff.head(2).norm() / max_lin_vel_;
    double dt = std::max(dt_rot, dt_lin);

    return dt;
}

void TimedElasticBand::initPrune(Eigen::Vector3f cur_pose) {
    if (pose_vec_.empty()) {
        return;
    }

    int nearest_idx = 0;
    float dist;
    float dist_cache = (cur_pose - pose_vec_[0]->pose().cast<float>()).head(2).squaredNorm();
    float angle_cache = uneven_planner::wrapToPi(cur_pose[2] - pose_vec_[0]->pose().cast<float>()[2]);

    // satisfy min_samples, otherwise max 10 samples
    int lookahead = pose_vec_.size() - 1;

    for (int i = 1; i <= lookahead; ++i) {
        dist = (cur_pose - pose_vec_[i]->pose().cast<float>()).head(2).squaredNorm();
        auto angle_dist = uneven_planner::wrapToPi(pose_vec_[i]->pose().cast<float>()[2] - cur_pose[2]);
        if (dist > dist_cache || abs(uneven_planner::wrapToPi(angle_dist - angle_cache)) > 10 / 180 * M_PI) {
            break;
        } else {
            dist_cache = dist;
            nearest_idx = i;
        }
    }
    nearest_idx -= 1;
    //	cout << "init_prune nearest_idx:" << nearest_idx
    //	     << " pose: " << pose_vec_[nearest_idx]->pose()[0]
    //	     <<" " << pose_vec_[nearest_idx]->pose()[1]
    //	     << "  dist:" << dist
    //	     << "  dist_cache:" << dist_cache
    //	     << " cur pose: "  << cur_pose[0]
    //	     << " " << cur_pose[1]
    //	     << endl;
    if (nearest_idx > 0) {
        deletePose(1, nearest_idx);
        deleteTimeDiff(1, nearest_idx);
    }

    pose_vec_[0]->pose() = cur_pose.cast<double>();
    if (pose_vec_.size() > 1) {
        time_diff_vec_[0]->dt() = calcTimeDiff(pose_vec_[0]->pose(), pose_vec_[1]->pose());
    }
}

void TimedElasticBand::adjustRotatePoints() {
    bool pure_rotate = false;
    Eigen::Vector3f point_rotate_pre = pose_vec_[0]->pose().cast<float>();
    Eigen::Vector3f point_rotate_start = point_rotate_pre;
    Eigen::Vector3f point_rotate_end;
    int rotate_start_idx = 0;
    for (int i = 1; i < pose_vec_.size() - 1; i++) {
        float dist_to_next =
            (pose_vec_[i + 1]->pose().head(2) - pose_vec_[i]->pose().head(2)).squaredNorm();
        if (dist_to_next < 1e-6) {
            if (!pure_rotate) {
                pure_rotate = true;
                point_rotate_pre = pose_vec_[i - 1]->pose().cast<float>();
                point_rotate_start = pose_vec_[i]->pose().cast<float>();
                rotate_start_idx = i;
            }
        } else {
            if (pure_rotate) {
                point_rotate_end = pose_vec_[i]->pose().cast<float>();
                float dist_prev_to_cur =
                    (point_rotate_end.head(2) - point_rotate_pre.head(2)).norm();
                //init pose rotate --startAlign ---startAlign:setFixed last point
                if (dist_prev_to_cur < 1e-6) {
                    pose_vec_[i]->setFixed(true);
                    pure_rotate = false;
                    continue;
                }
                Eigen::Vector3d insert_pose;
                insert_pose = pose_vec_.at(i)->pose();
                if (fabsf(point_rotate_start[2] - point_rotate_end[2]) < M_PI) {
                    insert_pose[2] = (point_rotate_start[2] + point_rotate_end[2]) / 2.0f;
                } else {
                    insert_pose[2] = (point_rotate_start[2] + point_rotate_end[2]) / 2.0f;
                    insert_pose[2] =
                        insert_pose[2] > 0 ? (-M_PI + insert_pose[2]) : (M_PI + insert_pose[2]);
                }
                deletePose(rotate_start_idx, i - rotate_start_idx + 1);
                deleteTimeDiff(rotate_start_idx, i - rotate_start_idx + 1);
                insertPose(rotate_start_idx, insert_pose);
                pose_vec_.at(rotate_start_idx)->setFixed(true);
                time_diff_vec_.at(rotate_start_idx - 1)->dt() =
                    calcTimeDiff(pose_vec_.at(rotate_start_idx - 1)->pose(),
                                 pose_vec_.at(rotate_start_idx)->pose());
                double insert_time = calcTimeDiff(pose_vec_.at(rotate_start_idx)->pose(),
                                                  pose_vec_.at(rotate_start_idx + 1)->pose());
                insertTimeDiff(rotate_start_idx, insert_time);
                //				cout << "delete from: " << rotate_start_idx
                //				     << " to " << i
                //				     << " insert:" << insert_pose[0] << " "
                //				     << insert_pose[1] << " "
                //					 << insert_pose[2] << " insert time: "
                //				     << insert_time << " verify: "
                //				     << pose_vec_.at(rotate_start_idx)->pose()
                //				     << " pose size: " << pose_vec_.size() << endl;
                i = rotate_start_idx - 1;
            }
            pure_rotate = false;
        }
    }
}

float TimedElasticBand::poseDistance(const Eigen::Vector3f& pose_a, const Eigen::Vector3f& pose_b) {
#if 0
    return (pose_a - pose_b).head(2).squaredNorm();
#else
    return (pose_a - pose_b).head(2).squaredNorm()
        + std::fabs(uneven_planner::wrapToPi(pose_a[2] - pose_b[2])) * 1 / M_PI;
#endif
}

void TimedElasticBand::updateAndPruneTEB(const Eigen::Vector3f& pose, int min_samples) {
    if (pose_vec_.empty()) {
        return;
    }

#if 1

    int nearest_idx = 0;
    float dist;
    float dist_cache = poseDistance(pose, pose_vec_[0]->pose().cast<float>());
    if (pose_vec_.size() > 2) {
        dist_cache = poseDistance(pose, pose_vec_[1]->pose().cast<float>());
        nearest_idx = 1;
    }

    // satisfy min_samples, otherwise max 5 samples
    int lookahead = pose_vec_.size() - min_samples;

    for (int i = 1; i <= lookahead; ++i) {
        dist = poseDistance(pose, pose_vec_[i]->pose().cast<float>());
        if (dist < dist_cache) {
            dist_cache = dist;
            nearest_idx = i;
        }
        //        if ((pose_vec_[i]->pose().head(2) - pose_vec_[nearest_idx]->pose().head(2)).squaredNorm() < 1e-6
        //		        && i > nearest_idx) {
        //        	nearest_idx = i;
        //        }
    }

    //    cout << "nearest_idx:" << nearest_idx
    //	     << " pose: " << pose_vec_[nearest_idx]->pose()[0]
    //	     <<" " << pose_vec_[nearest_idx]->pose()[1]
    //         << "  dist:" << dist
    //         << "  dist_cache:" << dist_cache
    //	     << " cur pose: "  << pose[0]
    //	     << " " << pose[1]
    //         << endl;

#else

    int nearest_idx = 0;
    float dist_square_min = (pose - pose_vec_[0]->pose().cast<float>()).head(2).squaredNorm();

    // satisfy min_samples, otherwise max 10 samples
    int lookahead = std::min<int>(pose_vec_.size() - min_samples, 10);

    for (int i = 1; i <= lookahead; ++i) {
        float dist_square = (pose - pose_vec_[i]->pose().cast<float>()).head(2).squaredNorm();

        if (dist_square < dist_square_min) {
            dist_square_min = dist_square;
            nearest_idx = i;
        } else {
            break;
        }
    }

#endif  // 0

    //    cout << "updateAndPruneTEB:" << nearest_idx
    //         << "  cur_pose:(" << pose[0]
    //         << ", " << pose[1]
    //         << ", " << pose[2]
    //         << ") (" << pose_vec_[0]->pose()[0]
    //         << ", " << pose_vec_[0]->pose()[1]
    //         << ", " << pose_vec_[0]->pose()[2]
    //         << ")" << endl;
//    ALOGD("update and prune teb index: %d", nearest_idx);
    if (nearest_idx > 0) {
        deletePose(1, nearest_idx);
        deleteTimeDiff(1, nearest_idx);
        //        for (int j = 1; j < nearest_idx + 1; ++j) {
        //            deletePose(j);
        //            deleteTimeDiff(j);
        //        }
    }

    pose_vec_[0]->pose() = pose.cast<double>();
    //	cout << "first time diff: " << time_diff_vec_.at(0)->dt();
    time_diff_vec_[0]->dt() = calcTimeDiff(pose_vec_[0]->pose(), pose_vec_[1]->pose());
    //    cout << "   " << time_diff_vec_.at(0)->dt() << endl;
}

// todo: Resize collision check!!!
void TimedElasticBand::autoResize(float time_step) {
    int min_samples = 3;

    float dt_hysteresis = 0.1f;
    float lower_bound = time_step - dt_hysteresis;
    float upper_bound = time_step + dt_hysteresis;

    bool modified = true;

    int max_iterations = 20;
    // actually it should be while(), but we want to make sure to not get stuck in some oscillation, hence max 20 repetitions.
    for (int rep = 0; rep <= max_iterations && modified; ++rep) {
        modified = false;

#if 1
        for (int i = 0; i < time_diff_vec_.size(); ++i) {
            /* Insert pose */
            if (time_diff_vec_[i]->dt() > upper_bound) {
                double new_time = 0.5f * time_diff_vec_[i]->dt();
                time_diff_vec_[i]->dt() = new_time;

                //                cout << "insert " << i + 1 << " : pose:" << *pose_vec_[i + 1] << "  time_diff:" << new_time << endl;
                Eigen::Vector3d insert_pose =
                    (pose_vec_[i]->pose() + pose_vec_[i + 1]->pose()) / 2.0f;
                insert_pose[2] =
                    atan2((sin(pose_vec_[i]->pose()[2]) + sin(pose_vec_[i + 1]->pose()[2])),
                          (cos(pose_vec_[i]->pose()[2]) + cos(pose_vec_[i + 1]->pose()[2])));

                //                cout << "insert i:" << i
                //                     << " (" << insert_pose[0]
                //                     << ", " << insert_pose[1]
                //                     << ", " << insert_pose[2]
                //                     << ")" << endl;

                insertPose(i + 1, insert_pose);
                insertTimeDiff(i + 1, new_time);

                modified = true;

            } else if (time_diff_vec_[i]->dt() < lower_bound
                       && time_diff_vec_.size() > min_samples) {
                /* Remove pose */
                if (i < (time_diff_vec_.size() - 1)) {
                    time_diff_vec_[i + 1]->dt() =
                        time_diff_vec_[i + 1]->dt() + time_diff_vec_[i]->dt();
                    //                    cout << "delete " << i + 1 << " : pose:" << *pose_vec_[i + 1]
                    //                         << "  time_diff:" << time_diff_vec_[i]->dt() << " "
                    //                         << time_diff_vec_[i + 1]->dt() << endl;
                    deleteTimeDiff(i);
                    deletePose(i + 1);
                } else {
                    // last motion should be adjusted, shift time to the interval before
                    //                    cout << "delete " << i << " : pose:" << *pose_vec_[i]
                    //                         << "  time_diff:" << time_diff_vec_[i]->dt() << endl;
                    time_diff_vec_[i - 1]->dt() += time_diff_vec_[i]->dt();
                    deleteTimeDiff(i);
                    deletePose(i);
                }

                modified = true;
            }
        }
        //        if (!modified) {
        //            cout << "TimedElasticBand::autoResize not modified!" << endl;
        //        }
        //        if (rep == max_iterations) {
        //            cout << "TimedElasticBand::autoResize run to max_iterations:" << max_iterations << "!" << endl;
        //        }

#else
        cout << "----> autoResize <----" << endl;
        for (int i = 0; i < time_diff_vec_.size(); ++i) {
            /**
             * < lower_bound
             *      not final pose: update next
             *      final pose: update pre
             * > upper_bound
             *      interpolation
             */
            cout << "dt " << i << " : " << time_diff_vec_[i]->dt() << endl;
            if (time_diff_vec_[i]->dt() < lower_bound) {
                if (i < time_diff_vec_.size() - 1) {
                    time_diff_vec_[i + 1]->dt() =
                        time_diff_vec_[i + 1]->dt() + time_diff_vec_[i]->dt();

                    cout << "delete " << i << " : pose:" << *pose_vec_[i + 1]
                         << "  time_diff:" << time_diff_vec_[i]->dt() << endl;

                    deleteTimeDiff(i);
                    deletePose(i + 1);
                    i -= 1;
                } else {
                    time_diff_vec_[i - 1]->dt() += time_diff_vec_[i]->dt();
                    deleteTimeDiff(i);
                    deletePose(i + 1);
                }
                modified = true;
            } else if (time_diff_vec_[i]->dt() > upper_bound) {
                double ratio = time_step / time_diff_vec_[i]->dt();
                int cnt = static_cast<int>(trunc(time_diff_vec_[i]->dt() / time_step));
                double remain_time = time_diff_vec_[i]->dt() - (double)cnt * time_step;

                cout << "ratio: " << ratio << " cnt:" << cnt << endl;

                double scale;
                Eigen::Vector3d start_inter_pose = pose_vec_[i]->pose();
                Eigen::Vector3d end_inter_pose = pose_vec_[i + 1]->pose();

                time_diff_vec_[i]->dt() = time_step;
                for (int j = 1; j <= cnt; ++j) {
                    scale = ratio * j;
                    Eigen::Vector3d new_pose =
                        start_inter_pose * (1.0f - scale) + end_inter_pose * scale;
                    insertPose(i + j, new_pose);

                    double new_time;
                    if (j < cnt) {
                        new_time = time_step;
                    } else if (j == cnt) {
                        new_time = remain_time;
                    }
                    insertTimeDiff(i + j, new_time);
                    cout << "insert " << i + j << " : pose:" << *pose_vec_[i + j]
                         << "  time_diff:" << new_time << endl;
                }

                i += cnt;
                modified = true;
            }
        }
        cout << "pose_vec_ size:" << pose_vec_.size() << " final:" << *pose_vec_.back() << endl;
#endif  //0
    }

    //    int cnt = 0;
    //    for (const auto &pose:pose_vec_) {
    //        cout << "pose_vec_:" << cnt
    //             << " " << *pose;
    //        if (cnt < time_diff_vec_.size()) {
    //            cout << "  time_diff_vec_:" << cnt
    //                 << " " << time_diff_vec_[cnt]->dt()
    //                 << endl;
    //        }
    //        cnt++;
    //    }
    //    cout << endl;
}

void TimedElasticBand::clearVertexSequences() {
    for (auto& pose : pose_vec_) {
        delete pose;
    }
    pose_vec_.clear();

    for (auto& time_diff : time_diff_vec_) {
        delete time_diff;
    }
    time_diff_vec_.clear();
}

float TimedElasticBand::getSumOfAllTimeDiffs() {
    float time = 0.0f;

    for (auto time_diff : time_diff_vec_) {
        time += time_diff->dt();
    }
    return time;
}
