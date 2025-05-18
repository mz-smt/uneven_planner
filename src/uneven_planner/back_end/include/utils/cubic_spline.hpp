//
// Created by mazheng on 25-5-15.
//

#ifndef SRC_CUBIC_SPLINE_HPP
#define SRC_CUBIC_SPLINE_HPP
#include <Eigen/Core>
#include <cmath>
#include <vector>
#include <algorithm>

namespace uneven_planner {
    template <typename T>
    const T& clamp(const T& v, const T& lo, const T& hi) {
        return (v < lo) ? lo : (v > hi) ? hi : v;
    }
    class SE2CubicSplineC2 {
    public:
        SE2CubicSplineC2(const Eigen::Vector3f& start, const Eigen::Vector3f& end,
                         float v0=1.0f, float v1=1.0f)
        {
            // 边界位置
            float x0=start.x(), y0=start.y(), x1=end.x(), y1=end.y();
            // 切线分量
            float dx0=v0*cos(start.z()), dy0=v0*sin(start.z());
            float dx1=v1*cos(end.z()),   dy1=v1*sin(end.z());
            // 构造矩阵 M 与向量 rhs
            Eigen::Matrix4f M; M <<
                                 1,0,  0,   0,
                    1,1,  1,   1,
                    0,1,  0,   0,
                    0,1,  2,   3;
            Eigen::Vector4f rhsx(x0, x1, dx0, dx1),
                    rhsy(y0, y1, dy0, dy1);
            // 求解系数
            Eigen::Vector4f ax = M.inverse()*rhsx;
            Eigen::Vector4f by = M.inverse()*rhsy;
            a0=ax(0); a1=ax(1); a2=ax(2); a3=ax(3);
            b0=by(0); b1=by(1); b2=by(2); b3=by(3);
        }

        /// 计算插值位姿
        Eigen::Vector3f evaluate(float t) const {
            t = clamp(t,0.0f,1.0f);
            float t2=t*t, t3=t2*t;
            float x = a0 + a1*t + a2*t2 + a3*t3;
            float y = b0 + b1*t + b2*t2 + b3*t3;
            // 导数
            float dx = a1 + 2*a2*t + 3*a3*t2;
            float dy = b1 + 2*b2*t + 3*b3*t2;
            float theta = std::atan2(dy, dx);
            return {x, y, theta};
        }

        /// 等间隔采样 N+1 点
        std::vector<Eigen::Vector3f> sample(int N) const {
            std::vector<Eigen::Vector3f> pts; pts.reserve(N+1);
            for(int i=0;i<=N;++i) pts.push_back(evaluate(float(i)/N));
            return pts;
        }

    private:
        // x(t)=a0 + a1 t + a2 t^2 + a3 t^3
        float a0,a1,a2,a3, b0,b1,b2,b3;
    };
}

#endif //SRC_CUBIC_SPLINE_HPP
