//
// Created by Fabian Meyer on 2023-02-13.
//

#include "Solution.h"

fzi::top_uav::Solution::Solution()
        : time_optimal_trajectory_duration(-1.0)
        , acceleration_profiles({Acceleration_Profile(), Acceleration_Profile(), Acceleration_Profile()})
{
}

const std::vector<fzi::top_uav::Acceleration_Profile>& fzi::top_uav::Solution::get_acceleration_profiles() const
{
    return acceleration_profiles;
}

void fzi::top_uav::Solution::set_acceleration_profile(const double& a_1, const double& a_2, const double& a_3, const double& t_1, const double& t_2, const double& t_3, const char& axis)
{
    switch (axis)
    {
        case 'x':
            acceleration_profiles[0].set_acceleration_segments(a_1, a_2, a_3);
            acceleration_profiles[0].set_time_durations_segments(t_1, t_2, t_3);
            break;
        case 'y':
            acceleration_profiles[1].set_acceleration_segments(a_1, a_2, a_3);
            acceleration_profiles[1].set_time_durations_segments(t_1, t_2, t_3);
            break;
        case 'z':
            acceleration_profiles[2].set_acceleration_segments(a_1, a_2, a_3);
            acceleration_profiles[2].set_time_durations_segments(t_1, t_2, t_3);
            break;
    }
}

void fzi::top_uav::Solution::set_acceleration_profile(const Acceleration_Profile& acceleration_profile, const char& axis)
{
    switch (axis)
    {
        case 'x':
            acceleration_profiles[0].set_acceleration_segments(acceleration_profile.get_accelerations_segments());
            acceleration_profiles[0].set_time_durations_segments(acceleration_profile.get_time_durations_segments());
            break;
        case 'y':
            acceleration_profiles[1].set_acceleration_segments(acceleration_profile.get_accelerations_segments());
            acceleration_profiles[1].set_time_durations_segments(acceleration_profile.get_time_durations_segments());
            break;
        case 'z':
            acceleration_profiles[2].set_acceleration_segments(acceleration_profile.get_accelerations_segments());
            acceleration_profiles[2].set_time_durations_segments(acceleration_profile.get_time_durations_segments());
            break;
    }
}

void fzi::top_uav::Solution::copy(const Solution& solution)
{
    time_optimal_trajectory_duration = solution.get_time_optimal_trajectory_duration();
    set_acceleration_profile(solution.get_acceleration_profiles()[0], 'x');
    set_acceleration_profile(solution.get_acceleration_profiles()[1], 'y');
    set_acceleration_profile(solution.get_acceleration_profiles()[2], 'z');
}
