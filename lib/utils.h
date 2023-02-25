//
// Created by Fabian Meyer on 2023-02-13.
//

#ifndef TRAJECTORY_GENERATION_LIB_UTILS_H
#define TRAJECTORY_GENERATION_LIB_UTILS_H
#pragma once
#include <cmath>
#include <cstdint>

inline double round_customized(const double& value) {
    return (std::int64_t)(value * 1000000.0) / 1000000.0;
};

inline double pow2(const double& value) {
    return value * value;
};

inline double sqrt2(const double& value)
{
    return sqrt(value);
}
#endif //TRAJECTORY_GENERATION_LIB_UTILS_H
