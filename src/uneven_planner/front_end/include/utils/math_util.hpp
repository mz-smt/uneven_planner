//
// Created by mazheng on 25-5-8.
//

#ifndef SRC_MATH_UTIL_HPP
#define SRC_MATH_UTIL_HPP

#include <Eigen/Eigen>
#include <cassert>
#include <cmath>
#include <functional>
#include <iostream>

namespace uneven_planner {

    struct VehicleParams {
        double mass;  // 车辆质量 m
        double wheelbase;  // 轴距 L (前后轮间距)
        double trackWidth;  // 轮距 W (左右轮间距)
        double cgHeight;  // 重心高度 h
        double g;  // 重力加速度 (例如 9.81 m/s²)
        double rearWeightFraction;  // 后轴载重比例 k (例如 0.6)
    };

    inline std::string str_format(const char* format, ...) {
        va_list args;
        va_start(args, format);
        size_t len = std::vsnprintf(NULL, 0, format, args);
        va_end(args);
        std::vector<char> vec(len + 1);
        va_start(args, format);
        std::vsnprintf(&vec[0], len + 1, format, args);
        va_end(args);
        return &vec[0];
    }

    inline float cos_approximation(float x) {
        constexpr auto c1 = 0.9999932946f;
        constexpr auto c2 = -0.4999124376f;
        constexpr auto c3 = 0.0414877472f;
        constexpr auto c4 = -0.0012712095f;
        float x2 = x * x;  // The input argument squared
        return (c1 + x2 * (c2 + x2 * (c3 + c4 * x2)));
    }

    inline float fast_cos(float angle) {
        constexpr auto InvTwoPi = 0.1591549f;
        constexpr auto TwoPi = 6.283185f;
        constexpr auto ThreeHalfPi = 4.7123889f;

        //clamp to the range 0..2pi
        angle = angle - floorf(angle * InvTwoPi) * TwoPi;
        angle = angle > 0.f ? angle : -angle;

        if (angle < M_PI_2) return cos_approximation(angle);
        if (angle < M_PI) return -cos_approximation(M_PI - angle);
        if (angle < ThreeHalfPi) return -cos_approximation(angle - M_PI);
        return cos_approximation(TwoPi - angle);
    }

    inline float fast_sin(float angle) {
        return fast_cos(M_PI_2 - angle);
    }

    // fast atan() :https://mazzo.li/posts/vectorized-atan2.html#atan2-what
    inline float atan_approximation(float x) {
        constexpr auto a1 = 0.99997726f;
        constexpr auto a3 = -0.33262347f;
        constexpr auto a5 = 0.19354346f;
        constexpr auto a7 = -0.11643287f;
        constexpr auto a9 = 0.05265332f;
        constexpr auto a11 = -0.01172120f;
        float x_sq = x * x;
        return x * (a1 + x_sq * (a3 + x_sq * (a5 + x_sq * (a7 + x_sq * (a9 + x_sq * a11)))));
    }

    inline float fast_atan2(float y, float x) {
        bool swap = fabs(x) < fabs(y);
        float atan_input = (swap ? x : y) / (swap ? y : x);
        // Approximate atan
        float res = atan_approximation(atan_input);

        // If swapped, adjust atan output
        res = swap ? (atan_input >= 0.0f ? M_PI_2 : -M_PI_2) - res : res;
        // Adjust quadrants
        if (x >= 0.0f && y >= 0.0f) {
        }  // 1st quadrant
        else if (x < 0.0f && y >= 0.0f) {
            res = M_PI + res;
        }  // 2nd quadrant
        else if (x < 0.0f && y < 0.0f) {
            res = -M_PI + res;
        }  // 3rd quadrant
        else if (x >= 0.0f && y < 0.0f) {}  // 4th quadrant

        return res;
    }

    //
    inline double fastPow(double a, int b) {
        switch (b) {
            case 0:
                return 1.0;
            case 1:
                return a;
            case 2:
                return a * a;
            case 3:
                return a * a * a;
            case 4: {
                double temp = a * a;
                return temp * temp;
            }
            case 5: {
                double temp = a * a;
                return temp * temp * a;
            }
            default:
                return std::pow(a, b);
        }
    }

    //        // https://martin.ankerl.com/2012/01/25/optimized-approximative-pow-in-c-and-cpp/
    //        inline double fastPow(double a, double b) {
    //            union {
    //                double d;
    //                int x[2];
    //            } u = { a };
    //            u.x[1] = (int)(b * (u.x[1] - 1072632447) + 1072632447);
    //            u.x[0] = 0;
    //            return u.d;
    //        }
    //
    //        // should be much more precise with large b
    //        inline double fastPrecisePow(double a, double b) {
    //            // calculate approximation with fraction of the exponent
    //            int e = (int) b;
    //            union {
    //                double d;
    //                int x[2];
    //            } u = { a };
    //            u.x[1] = (int)((b - e) * (u.x[1] - 1072632447) + 1072632447);
    //            u.x[0] = 0;
    //
    //            // exponentiation by squaring with the exponent's integer part
    //            // double r = u.d makes everything much slower, not sure why
    //            double r = 1.0;
    //            while (e) {
    //                if (e & 1) {
    //                    r *= a;
    //                }
    //                a *= a;
    //                e >>= 1;
    //            }
    //
    //            return r * u.d;
    //        }
    template<typename T>
    struct is_float_or_double : std::false_type {};
    template<>
    struct is_float_or_double<float> : std::true_type {};
    template<>
    struct is_float_or_double<double> : std::true_type {};

    template<typename T, typename = std::enable_if_t<is_float_or_double<T>::value>>
    constexpr inline T wrapTo2Pi(const T& a) noexcept {
        constexpr T TWO_PI = static_cast<T>(2.0 * M_PI);
        T r = std::fmod(a, TWO_PI);
        return r < 0 ? r + TWO_PI : r;
    }
    template<typename T, typename = std::enable_if_t<is_float_or_double<T>::value>>
    constexpr inline T wrapToPi(const T& a) noexcept {
        constexpr T TWO_PI = static_cast<T>(2.0 * M_PI);
        constexpr T PI = static_cast<T>(M_PI);
        T r = std::fmod(a + PI, TWO_PI);
        return r < 0 ? r + PI : r - PI;
    }
    template<typename T, typename = std::enable_if_t<is_float_or_double<T>::value>>
    inline T degToRad(const T& a) {
        return wrapTo2Pi(a) * 0.0174532925f;
    }
    template<typename T, typename = std::enable_if_t<is_float_or_double<T>::value>>
    inline T radToDeg(const T& a) {
        return wrapTo2Pi(a) * 57.2957795f;
    }

    inline std::pair<double, double> cartesian2Polar(double x, double y) {
        double r = std::sqrt(x * x + y * y);
        double theta = std::atan2(y, x);
        return std::make_pair(r, theta);
    }

    //叉乘 检查点C与line AB的位置关系。 0 on line; >0 up line; <0 down line
    inline float cross(const Eigen::Vector2f& point_a, const Eigen::Vector2f& point_b,
                       const Eigen::Vector2f& point_c) {
        return (point_c[0] - point_a[0]) * (point_c[1] - point_b[1])
               - (point_c[1] - point_a[1]) * (point_c[0] - point_b[0]);
    }
    //        inline float cross(Eigen::Vector3f &point_a, Eigen::Vector3f &point_b, Eigen::Vector3f &point_c) {
    //            return cross(point_a.head(2), point_b.head(2), point_c.head(2));
    //        }

    //求向量v1 v2的夹角, 注:范围[0, M_PI]
    inline float vectorAngle(const Eigen::Vector2f& v1, const Eigen::Vector2f& v2) {
        // std::acos domain error occurs if arg is outside the range [-1.0, 1.0]
        // a domain error occurs and NaN is returned直接用 return r < 0 ? r + PI : r - PI; 可以吗
        double value = v1.dot(v2) / (v1.norm() * v2.norm());
        value = value > 1.0 ? 1.0 : (value < -1.0 ? -1.0 : value);
        return std::acos(value);
    }

    //求点c到直线ab的垂直距离
    inline float distanceToLine(const Eigen::Vector2f& point_a, const Eigen::Vector2f& point_b,
                                const Eigen::Vector2f& point_c) {
        float A = point_b[1] - point_a[1];
        float B = point_a[0] - point_b[0];
        float C = -A * point_a[0] - B * point_a[1];
        return std::abs(A * point_c[0] + B * point_c[1] + C) / std::sqrt(A * A + B * B);
    }

    //求点c到直线ab的垂足
    inline Eigen::Vector2f footPoint(const Eigen::Vector2f& point_a, const Eigen::Vector2f& point_b,
                                     const Eigen::Vector2f& point_c) {
        float A = point_b[1] - point_a[1];
        float B = point_a[0] - point_b[0];
        float C = -A * point_a[0] - B * point_a[1];
        Eigen::Vector2f foot_point;
        foot_point[0] = (B * B * point_c[0] - A * B * point_c[1] - A * C) / (A * A + B * B);
        foot_point[1] = (-A * B * point_c[0] + A * A * point_c[1] - B * C) / (A * A + B * B);
        return foot_point;
    }

    //求点point到直线line的垂足
    inline Eigen::Vector2f footPoint(const Eigen::Vector3f& line, const Eigen::Vector2f& point) {
        Eigen::Vector2f point_a = line.head(2);
        Eigen::Vector2f point_b(line.x() + 1.0f * std::cos(line[2]),
                                line.y() + 1.0f * std::sin(line[2]));
        return footPoint(point_a, point_b, point);
    }

    // return point distance from A dis's point
    inline Eigen::Vector2f calculatePerpendicularPoint(const Eigen::Vector2f& origin_point,
                                                       float point_theta, float distance) {
        Eigen::Vector2f p(-std::cos(point_theta), -std::sin(point_theta));
        Eigen::Vector2f n(-p[1], p[0]);
        return origin_point + (distance * n);
    }

    inline Eigen::Vector3f calculatePerpendicularPoint(const Eigen::Vector3f& origin_point,
                                                       float distance) {
        auto temp_out_point =
                calculatePerpendicularPoint(origin_point.head(2), origin_point[2], distance);
        return Eigen::Vector3f(temp_out_point[0], temp_out_point[1], origin_point[2]);
    }

    //计算某点沿圆心旋转固定弧长后的新点
    inline Eigen::Vector3f calculateRotatePoint(const Eigen::Vector2f& center,
                                                const Eigen::Vector3f& point,
                                                const double& arc_length) {
        // 计算半径 r
        double r =
                std::sqrt(std::pow(point.x() - center.x(), 2) + std::pow(point.y() - center.y(), 2));

        // 计算初始角度 theta
        double theta = std::atan2(point.y() - center.y(), point.x() - center.x());

        // 计算旋转角度 delta_theta
        double delta_theta = arc_length / r;

        Eigen::Vector3f new_point;
        // 计算新点 C 的坐标
        new_point.x() = center.x() + r * std::cos(theta + delta_theta);
        new_point.y() = center.y() + r * std::sin(theta + delta_theta);
        new_point.z() = wrapToPi(point[2] + delta_theta);

        return new_point;
    }

    //求点pt到线段p1 p2的最短距离 http://www.lu16.com/?id=4
    template<typename Point = Eigen::Vector2f>
    inline float pointToSegDist(const Point& p1, const Point& p2, const Point& pt) {
        float fDot = (p2.x() - p1.x()) * (pt.x() - p1.x()) + (p2.y() - p1.y()) * (pt.y() - p1.y());
        if (fDot <= 0.0f) {
            return sqrt((p1.x() - pt.x()) * (p1.x() - pt.x())
                        + (p1.y() - pt.y()) * (p1.y() - pt.y()));
        }

        float d2AB = (p1.x() - p2.x()) * (p1.x() - p2.x()) + (p1.y() - p2.y()) * (p1.y() - p2.y());
        if (fDot >= d2AB) {
            return sqrt((p2.x() - pt.x()) * (p2.x() - pt.x())
                        + (p2.y() - pt.y()) * (p2.y() - pt.y()));
        }

        float u = fDot / d2AB;
        float AC_x = p1.x() + (p2.x() - p1.x()) * u;
        float AC_y = p1.y() + (p2.y() - p1.y()) * u;
        return sqrt((pt.x() - AC_x) * (pt.x() - AC_x) + (pt.y() - AC_y) * (pt.y() - AC_y));
    }

    //求点pt到逆时针顺序存储边界的最短距离，返回垂足，垂足后方边界点的index
    inline bool nearestPointInfoToPolygon(const std::vector<Eigen::Vector2f>& polygon,
                                          const Eigen::Vector2f& point,
                                          Eigen::Vector2f& nearest_point, int& front_id) {
        float min_dist = std::numeric_limits<float>::max();
        front_id = -1;
        for (int i = 0; i < polygon.size(); ++i) {
            auto point1 = polygon[i];
            auto point2 = polygon[(i + 1) % polygon.size()];
            auto dist = pointToSegDist(point1, point2, point);
            if (dist < min_dist) {
                min_dist = dist;
                front_id = i;
            }
        }
        if (front_id == -1) return false;
        auto foot_point =
                footPoint(polygon[front_id], polygon[(front_id + 1) % polygon.size()], point);
        // 判断垂足是否在线段上。不在线段上，则取最近点为端点
        auto line_length = (polygon[front_id] - polygon[(front_id + 1) % polygon.size()]).norm();
        auto dist1 = (foot_point - polygon[front_id]).norm();
        auto dist2 = (foot_point - polygon[(front_id + 1) % polygon.size()]).norm();
        if (dist1 + dist2 - line_length < 0.001f) {
            nearest_point = std::move(foot_point);
            return true;
        }
        nearest_point =
                dist1 < dist2 ? polygon[front_id] : polygon[(front_id + 1) % polygon.size()];
        return true;
    }

    template<typename T>
    inline bool calcuTwoSegmentIntersection(const std::vector<Eigen::Vector2f>& segment1,
                                            const std::vector<T>& segment2,
                                            Eigen::Vector2f& intersect_point) {
        if (segment1.size() != 2 || segment2.size() != 2) return false;

        auto dx1 = segment1.back().x() - segment1.front().x();
        auto dy1 = segment1.back().y() - segment1.front().y();
        auto dx2 = segment2.back().x() - segment2.front().x();
        auto dy2 = segment2.back().y() - segment2.front().y();
        double cross = dx1 * dy2 - dy1 * dx2;
        if (std::abs(cross) < 1e-8) {
            // 线段平行或共线，没有交点
            return false;  // 返回一个无效的点表示没有交点
        }

        auto t1 = ((segment2.front().x() - segment1.front().x()) * dy2
                   - (segment2.front().y() - segment1.front().y()) * dx2)
                  / cross;
        auto t2 = ((segment2.front().x() - segment1.front().x()) * dy1
                   - (segment2.front().y() - segment1.front().y()) * dx1)
                  / cross;

        if (t1 >= 0 && t1 <= 1 && t2 >= 0 && t2 <= 1) {
            // 确保交点在两个线段上
            intersect_point.x() = segment1.front().x() + t1 * dx1;
            intersect_point.y() = segment1.front().y() + t1 * dy1;
        } else {
            // 线段相交但交点不在两个线段上
            return false;  // 返回一个无效的点表示没有交点
        }
        return true;
    }

    class QuadPoly {
    public:
        volatile float a, b, c;
        static const char* TAG;

        QuadPoly(float a, float b, float c) {
            setPoly(a, b, c);
        }

        ~QuadPoly() {};

        void setPoly(float a_, float b_, float c_) {
            a = a_;
            b = b_;
            c = c_;
        }

        float max() {
            return (4 * a * c - b * b) / (2 * a);
        }

        float rootPos() {
            //        LOGD("QuadPoly::rootPos");
            float s = getSqrItem();
            //!! Important fix for clang!
            //!! if a is very small but not zero, O2 solver will always return 0.
            if (fabsf(a) <= 0.0001f) {
                if (b != 0) {
                    float solve_x = -c / b;
                    //                LOGD("O1 Solve: %f", solve_x);
                    return solve_x;
                }
                //            LOGD("NAN 0");
                return NAN;
            }

            if (std::isnan(s) == 0) {
                float solve_x = (float)((-b + sqrt(s)) / (2 * a));
                //            LOGD("O2 Solve: %f, [a:%f, b:%f, c:%f, sqrt(s):%f]",
                //                 solve_x, a, b, c, sqrt(s));
                return solve_x;
            }
            //        LOGD("NAN 1");
            return NAN;
        }

        float getSqrItem() {
            float tmp = (b * b - 4 * a * c);

            if (tmp < 0) {
                //            std::cout << "No real root.\n";
                //            toSrting();
                return NAN;
            }
            return tmp;
        }

        float eval(float x) {
            return a * x * x + b * x + c;
        }

        void toSrting() {
            std::cout << " a = " << a << "; b = " << b << "; c = " << c << std::endl;
        }
    };

    inline float FastInvSqrtF32(float x) {
        long i;
        float x2, y;
        const float threehalfs = 1.5F;

        x2 = x * 0.5F;
        y = x;
        i = *(long*)&y;  // evil floating point bit level hacking
        i = 0x5f3759df - (i >> 1);  // what the fuck?
        y = *(float*)&i;
        y = y * (threehalfs - (x2 * y * y));  // 1st iteration
        //	y  = y * ( threehalfs - ( x2 * y * y ) );   // 2nd iteration, this can be removed

        return y;
    }

    inline float FastSqrtF32(float x) {
        return 1.0f / FastInvSqrtF32(x);
    }

    inline float NumDiffO1(std::function<float(float)>& f, float point, float inc) {
        return (f(point + inc) - f(point)) / inc;
    }

    inline float NumDiffO2(std::function<float(float)>& f, float point, float inc) {
        float g1 = (f(point + inc) - f(point)) / inc;
        float g2 = (f(point) - f(point - inc)) / inc;

        return (g2 - g1) / inc;
    }

#define MIN(a, b) ((a) > (b) ? (b) : (a))
#define MAX(a, b) ((a) < (b) ? (b) : (a))
#define ARRAY_LEN(arr) (sizeof(arr) / sizeof(arr[0]))
#define SQR_SUM(a, b) ((a) * (a) + (b) * (b))

    inline float HuberLoss(float x, float thres = 1.0) {
        if (std::fabs(x) < thres) {
            return x * x;
        }
        return (std::fabs(x) - thres) * 2.0f * thres + (thres * thres);
    }

    typedef struct {
        float range[2];
        int n;
    } Linspace;

    inline std::vector<float> linspace(Linspace& input) {
        assert(input.range[1] >= input.range[0]);

        std::vector<float> result;

        float step = 0;

        if (input.n > 1) {
            step = (input.range[1] - input.range[0]) / (float)(input.n - 1);
        }

        float x = input.range[0];

        for (int i = 0; i < input.n; i++, x += step) {
            result.emplace_back(x);
        }

        return result;
    }

    inline Eigen::Vector2f rotateVector2D(const Eigen::Vector2f& vec, float theta) {

        //        const auto sin_theta = motion_planner::fast_sin(theta);
        //        const auto cos_theta = motion_planner::fast_cos(theta);
        const auto sin_theta = std::sin(theta);
        const auto cos_theta = std::cos(theta);

        Eigen::Vector2f result;
        result[0] = cos_theta * vec[0] - sin_theta * vec[1];
        result[1] = sin_theta * vec[0] + cos_theta * vec[1];
        return result;
    }

    inline Eigen::Vector3f groundFrameToBodyFrame(const Eigen::Vector3f& query,
                                                  const Eigen::Vector3f& cur_pose) {
        Eigen::Vector3f pose;
        pose.head(2) = rotateVector2D(query.head(2) - cur_pose.head(2), -cur_pose[2]);
        pose[2] = (float)wrapToPi(query[2] - cur_pose[2]);
        return pose;
    }
}

#endif //SRC_MATH_UTIL_HPP
