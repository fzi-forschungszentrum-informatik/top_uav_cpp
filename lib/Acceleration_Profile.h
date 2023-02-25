//
// Created by Fabian Meyer on 2023-02-13.
//

#ifndef TRAJECTORY_GENERATION_LIB_ACCELERATION_PROFILE_H
#define TRAJECTORY_GENERATION_LIB_ACCELERATION_PROFILE_H
#pragma once
#include <vector>

namespace fzi {
    namespace top_uav {

        class Acceleration_Profile {
        public:
            Acceleration_Profile();
            void set_acceleration_segments(const double& a_1, const double& a_2, const double& a_3);
            void set_time_durations_segments(const double& t_1, const double& t_2, const double& t_3);
            void set_acceleration_segments(const std::vector<double>& acceleration_segments);
            void set_time_durations_segments(const std::vector<double>& time_segments);
            const std::vector<double>& get_accelerations_segments() const { return accelerations_segments; };
            const std::vector<double>& get_time_durations_segments() const { return time_durations_segments; };
        private:
            std::vector<double> accelerations_segments;
            std::vector<double> time_durations_segments;
        };

    } // fzi
} // top_uav

#endif //TRAJECTORY_GENERATION_LIB_ACCELERATION_PROFILE_H
