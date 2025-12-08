//
// Created by mazheng on 25-5-8.
//

#ifndef SRC_SIMPLE_PATH_HPP
#define SRC_SIMPLE_PATH_HPP

#include <Eigen/Dense>
#include <memory>
#include <vector>
#include <ros/ros.h>
#include <utils/math_util.hpp>

namespace uneven_planner {
    class SimplePath {
    public:
        explicit SimplePath(float rotate_step_ = M_PI / 18.0f,  // 10 deg
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

        void setForbiddenDir(const bool& set_forbid_dir, const float& forbidden_dir) {
            set_forbid_dir_ = set_forbid_dir;
            forbidden_dir_ = forbidden_dir;
        }

        // 判断 x 是否在 (from, to) 这个有向开区间里
        bool angleInOpenInterval(float from, float to, float x) {
            // 把它们都归一
            from = wrapToPi(from);
            to   = wrapToPi(to);
            x    = wrapToPi(x);
            // 计算从 from 到 to 的 CCW 增量
            float d = wrapToPi(to - from);
            // 计算从 from 到 x 的 CCW 增量
            float dx = wrapToPi(x - from);
            // 如果 d>0，(from→to) 是 CCW；如果 d<0，(from→to) 是 CW
            if (d > 0) {
                // CCW: x 在区间里当且仅当 dx∈(0, d)
                return dx > 0 && dx < d;
            } else {
                // CW: 这时 to 在 CW 方向，因此对应 CCW 增量为 d (负数)
                // x 在区间里当且仅当 dx < 0 且 dx > d
                return dx < 0 && dx > d;
            }
        }

        // 主逻辑：判断最短旋转路径上是否经过 forbidden
        bool crossesForbidden(float start, float target, float forbidden) {
            start     = wrapToPi(start);
            target    = wrapToPi(target);
            forbidden = wrapToPi(forbidden);

            // CCW 增量
            float d_ccw = wrapToPi(target - start);
            // CW 增量 = d_ccw - sign(d_ccw)*2π
            float d_cw  = (d_ccw > 0 ? d_ccw - 2*M_PI : d_ccw + 2*M_PI);

            // 选择最短
            if (fabs(d_ccw) <= fabs(d_cw)) {
                // 逆时针更短或相等
                return angleInOpenInterval(start, start + d_ccw, forbidden);
            } else {
                // 顺时针更短
                return angleInOpenInterval(start, start + d_cw, forbidden);
            }
        }
    private:
        float ROTATE_STEP;
        float MOVE_STEP;
        bool force_forward = true;
        bool direction;
        float steer = 0.0f;
        bool theta_free = false;
        ros::Publisher simple_path_pub_;
        bool set_forbid_dir_ = false;
        float forbidden_dir_ = 0.0f;
    };
}



#endif //SRC_SIMPLE_PATH_HPP
