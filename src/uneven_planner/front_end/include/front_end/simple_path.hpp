//
// Created by mazheng on 25-5-8.
//

#ifndef SRC_SIMPLE_PATH_HPP
#define SRC_SIMPLE_PATH_HPP

#include <Eigen/Dense>
#include <memory>
#include <vector>
#include <ros/ros.h>

namespace uneven_planner {
    class SimplePath {
    public:
        explicit SimplePath(float rotate_step_ = M_PI / 36.0f,  // 5 deg
                            float move_step_ = 0.2f, bool force_forward_ = true);

        ~SimplePath() = default;

        void init(ros::NodeHandle& nh);

        bool generatePath(const Eigen::Vector3f& start_pt, const Eigen::Vector3f& end_pt,
                          std::vector<Eigen::Vector3f>& optimal_path);

        void setSampleParam(float rotate_step, float move_step);

        bool getDirec() const {
            return direction;
        }

        float getSteer() const {
            return steer;
        }

        void setThetaFree(bool is_theta_free) {
            theta_free = is_theta_free;
        }

        void setForceForward(bool is_force_forward) {
            force_forward = is_force_forward;
        }
    private:
        float ROTATE_STEP;
        float MOVE_STEP;
        bool force_forward = true;
        bool direction;
        float steer = 0.0f;
        bool theta_free = false;
        ros::Publisher simple_path_pub_;
    };
}



#endif //SRC_SIMPLE_PATH_HPP
