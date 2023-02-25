//
// Created by Fabian Meyer on 2023-02-13.
//

#ifndef TRAJECTORY_GENERATION_LIB_SOLUTION_H
#define TRAJECTORY_GENERATION_LIB_SOLUTION_H
#include "Acceleration_Profile.h"
#include <vector>

namespace fzi {
    namespace top_uav {

        class Solution
        {
        public:
            Solution();
            const double& get_time_optimal_trajectory_duration() const { return time_optimal_trajectory_duration; };
            const std::vector<Acceleration_Profile>& get_acceleration_profiles() const ;
            void set_time_optimal_trajectory_duration(const double& new_time_optimal_trajectory_duration) { time_optimal_trajectory_duration = new_time_optimal_trajectory_duration; };
            void set_acceleration_profile(const double& a_1, const double& a_2, const double& a_3, const double& t_1, const double& t_2, const double& t_3, const char& axis);
            void set_acceleration_profile(const Acceleration_Profile& acceleration_profile, const char& axis);
            void copy(const Solution& solution);
        private:
            double time_optimal_trajectory_duration;
            std::vector<Acceleration_Profile> acceleration_profiles;
        };

    } // fzi
} // top_uav

#endif //TRAJECTORY_GENERATION_LIB_SOLUTION_H
