//
// Created by mazheng on 25-4-28.
//

#ifndef SRC_ETA3_SPLINE_HPP
#define SRC_ETA3_SPLINE_HPP

#include <cmath>

namespace uneven_planner {
    class Eta3Spline {
    public:
        /*
         * Input start and end state to construct n3-spline.
         * */
        explicit Eta3Spline(float start_x, float start_y, float start_theta, float start_v,
                            float start_w, float end_x, float end_y, float end_theta, float end_v,
                            float end_w) {
            // calculate η3-spline with a closed-form
            const auto& xA = start_x;
            const auto& yA = start_y;
            const auto& thetaA = start_theta;
            //            const auto &kA =
            //                    fabs(start_v) < 1e-5f ? (fabs(start_w) < 1e-5f ? 0.0f : std::numeric_limits<float>::max()) :
            //                    start_w / start_v;
            const auto& kA = std::fabs(start_v) < 1e-2f ? 0.0f : start_w / start_v;

            //        const auto &dkA = 0;
            const auto sinA = sin(thetaA);
            const auto cosA = cos(thetaA);

            const auto& xB = end_x;
            const auto& yB = end_y;
            const auto& thetaB = end_theta;
            //            const auto &kB =
            //                    fabs(end_v) < 1e-5f ? (fabs(end_w) < 1e-5f ? 0.0f : std::numeric_limits<float>::max()) :
            //                    end_w / end_v;
            const auto& kB = fabs(end_v) < 1e-2f ? 0.0f : end_w / end_v;

            //        const auto &dkB = 0;
            const auto sinB = sin(thetaB);
            const auto cosB = cos(thetaB);

            const auto dist = hypot(start_x - end_x, start_y - end_y);
            /// n can be adjusted. 1.0f is suboptimal param here.
            const auto n = 0.5f * dist;
            const auto square_n = n * n;

            const auto dx = xB - xA;
            const auto temp1 = square_n * kA * sinA;
            const auto temp2 = square_n * kB * sinB;
            const auto n_cosA = n * cosA;
            const auto n_cosB = n * cosB;
            a0 = xA;
            a1 = n_cosA;
            a2 = -temp1 / 2.0f;
            a3 = 0;
            a4 = 35.0f * dx - 20.0f * n_cosA + 5.0f * temp1 - 15.0f * n_cosB - 5.0f / 2.0f * temp2;
            a5 = -84.0f * dx + 45.0f * n_cosA - 10.0f * temp1 + 39.0f * n_cosB + 7.0f * temp2;
            a6 = 70.0f * dx - 36.0f * n_cosA + 15.0f / 2.0f * temp1 - 34.0f * n_cosB
                 - 13.0f / 2.0f * temp2;
            a7 = -20.0f * dx + 10.0f * n_cosA - 2.0f * temp1 + 10.0f * n_cosB + 2.0f * temp2;

            const auto dy = yB - yA;
            const auto temp3 = square_n * kA * cosA;
            const auto temp4 = square_n * kB * cosB;
            const auto n_sinA = n * sinA;
            const auto n_sinB = n * sinB;

            b0 = yA;
            b1 = n_sinA;
            b2 = temp3 / 2.0f;
            b3 = 0;
            b4 = 35.0f * dy - 20.0f * n_sinA - 5.0f * temp3 - 15.0f * n_sinB + 5.0f / 2.0f * temp4;
            b5 = -84.0f * dy + 45.0f * n_sinA + 10.0f * temp3 + 39.0f * n_sinB - 7.0f * temp4;
            b6 = 70.0f * dy - 36.0f * n_sinA - 15.0f / 2.0f * temp3 - 34.0f * n_sinB
                 + 13.0f / 2.0f * temp4;
            b7 = -20.0f * dy + 10.0f * n_sinA + 2.0f * temp3 + 10.0f * n_sinB - 2.0f * temp4;
        }

        ~Eta3Spline() = default;

        // u = [0~1]
        // TODO: use Horner's method : https://en.wikipedia.org/wiki/Horner%27s_method
        Eigen::Vector2f evaluate(float u) {
            float u_2 = u * u;
            float u_3 = u_2 * u;
            float u_4 = u_3 * u;
            float u_5 = u_4 * u;
            float u_6 = u_5 * u;
            float u_7 = u_6 * u;
            float xu =
                    a0 + a1 * u + a2 * u_2 + a3 * u_3 + a4 * u_4 + a5 * u_5 + a6 * u_6 + a7 * u_7;
            float yu =
                    b0 + b1 * u + b2 * u_2 + b3 * u_3 + b4 * u_4 + b5 * u_5 + b6 * u_6 + b7 * u_7;
            return {xu, yu};
        }

        Eigen::Vector3f evaluate3D(float u) {
            float u_2 = u * u;
            float u_3 = u_2 * u;
            float u_4 = u_3 * u;
            float u_5 = u_4 * u;
            float u_6 = u_5 * u;
            float u_7 = u_6 * u;
            float xu =
                    a0 + a1 * u + a2 * u_2 + a3 * u_3 + a4 * u_4 + a5 * u_5 + a6 * u_6 + a7 * u_7;
            float yu =
                    b0 + b1 * u + b2 * u_2 + b3 * u_3 + b4 * u_4 + b5 * u_5 + b6 * u_6 + b7 * u_7;

            float xv = a1 + 2 * a2 * u + 3 * a3 * u_2 + 4 * a4 * u_3 + 5 * a5 * u_4 + 6 * a6 * u_5
                       + 7 * a7 * u_6;
            float yv = b1 + 2 * b2 * u + 3 * b3 * u_2 + 4 * b4 * u_3 + 5 * b5 * u_4 + 6 * b6 * u_5
                       + 7 * b7 * u_6;
            return {xu, yu, std::atan2(yv, xv)};
            //            return {xu, yu, fast_atan2(yv, xv)};
        }

    private:
        //        Eigen::VectorXf start_state, end_state;
        float a0, a1{}, a2{}, a3, a4{}, a5{}, a6{}, a7{};
        float b0, b1{}, b2{}, b3, b4{}, b5{}, b6{}, b7{};
    };
}

#endif //SRC_ETA3_SPLINE_HPP
