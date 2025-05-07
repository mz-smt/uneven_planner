//
// Created by mazheng on 25-5-7.
//

#ifndef SRC_MATH_UTILS_HPP
#define SRC_MATH_UTILS_HPP

#include <cmath>

namespace uneven_planner {

    inline double wrapToPi(double angle) {
        angle = fmod(angle + M_PI, 2.0 * M_PI);
        if (angle <= 0.0)
            angle += 2.0 * M_PI;
        return angle - M_PI;
    }

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
}

#endif //SRC_MATH_UTILS_HPP
