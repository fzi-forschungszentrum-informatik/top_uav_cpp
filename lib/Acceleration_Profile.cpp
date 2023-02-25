//
// Created by Fabian Meyer on 2023-02-13.
//

#include "Acceleration_Profile.h"

fzi::top_uav::Acceleration_Profile::Acceleration_Profile()
        : accelerations_segments({-1,-1,-1})
        , time_durations_segments({-1,-1,-1})
{
}

void fzi::top_uav::Acceleration_Profile::set_acceleration_segments(const double& a_1, const double& a_2, const double& a_3)
{
    accelerations_segments[0] = a_1;
    accelerations_segments[1] = a_2;
    accelerations_segments[2] = a_3;
}

void fzi::top_uav::Acceleration_Profile::set_time_durations_segments(const double& t_1, const double& t_2, const double& t_3)
{
    time_durations_segments[0] = t_1;
    time_durations_segments[1] = t_2;
    time_durations_segments[2] = t_3;
}


void fzi::top_uav::Acceleration_Profile::set_acceleration_segments(const std::vector<double>& acceleration_segments)
{
    accelerations_segments[0] = acceleration_segments[0];
    accelerations_segments[1] = acceleration_segments[1];
    accelerations_segments[2] = acceleration_segments[2];
}

void fzi::top_uav::Acceleration_Profile::set_time_durations_segments(const std::vector<double>& time_segments)
{
    time_durations_segments[0] = time_segments[0];
    time_durations_segments[1] = time_segments[1];
    time_durations_segments[2] = time_segments[2];
}