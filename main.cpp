//
// Created by Fabian Meyer on 2023-02-13.
//

#include "Solution.h"
#include "Trajectory_Planner.h"
#include <iostream>
#include <string>


using namespace fzi::top_uav;

////////////////////////////////////////////////
/// EXAMPLE FROM MEYER and GLOCK 2023 //////////
////////////////////////////////////////////////
int main() {

    double v_max = 4.0;
    double a_max = 1.0;

    Trajectory_Planner traj_planner_sota(v_max, a_max, "sota");
    Trajectory_Planner traj_planner_basic(v_max, a_max, "basic");
    Trajectory_Planner traj_planner_improved(v_max, a_max, "improved");

    double px_s = 0.1;
    double py_s = 2.0;
    double pz_s = 4.3;
    double vx_s = 0.1;
    double vy_s = -1.9;
    double vz_s = -0.4;

    double px_e = 3.6;
    double py_e = 0.4;
    double pz_e = 2.6;
    double vx_e = 0.1;
    double vy_e = -1.8;
    double vz_e = 0.6;

    Solution t_opt_basic = traj_planner_basic.calc_opt_time(px_s, px_e, py_s, py_e, pz_s, pz_e, vx_s, vx_e, vy_s, vy_e,
                                                            vz_s, vz_e);
    Solution t_opt_improved = traj_planner_improved.calc_opt_time(px_s, px_e, py_s, py_e, pz_s, pz_e, vx_s, vx_e, vy_s,
                                                                  vy_e, vz_s, vz_e);
    Solution t_opt_sota = traj_planner_sota.calc_opt_time(px_s, px_e, py_s, py_e, pz_s, pz_e, vx_s, vx_e, vy_s, vy_e,
                                                          vz_s, vz_e);

    std::cout << "Optimal trajectory duration (SOTA): "
              << std::to_string(t_opt_sota.get_time_optimal_trajectory_duration()) << std::endl;
    std::cout << "____________________________________________" << std::endl;
    std::cout << "Optimal trajectory duration (TOP-UAV-basic): "
              << std::to_string(t_opt_basic.get_time_optimal_trajectory_duration()) << std::endl;
    std::cout << "Times x: [" <<
              std::to_string(t_opt_basic.get_acceleration_profiles()[0].get_time_durations_segments()[0]) << "," <<
              std::to_string(t_opt_basic.get_acceleration_profiles()[0].get_time_durations_segments()[1]) << "," <<
              std::to_string(t_opt_basic.get_acceleration_profiles()[0].get_time_durations_segments()[2]) << "]" <<
              " / Accelerations x: [" <<
              std::to_string(t_opt_basic.get_acceleration_profiles()[0].get_accelerations_segments()[0]) << "," <<
              std::to_string(t_opt_basic.get_acceleration_profiles()[0].get_accelerations_segments()[1]) << "," <<
              std::to_string(t_opt_basic.get_acceleration_profiles()[0].get_accelerations_segments()[2]) << "]"
              << std::endl;

    std::cout << "Times y: [" <<
              std::to_string(t_opt_basic.get_acceleration_profiles()[1].get_time_durations_segments()[0]) << "," <<
              std::to_string(t_opt_basic.get_acceleration_profiles()[1].get_time_durations_segments()[1]) << "," <<
              std::to_string(t_opt_basic.get_acceleration_profiles()[1].get_time_durations_segments()[2]) << "]" <<
              " / Accelerations y: [" <<
              std::to_string(t_opt_basic.get_acceleration_profiles()[1].get_accelerations_segments()[0]) << "," <<
              std::to_string(t_opt_basic.get_acceleration_profiles()[1].get_accelerations_segments()[1]) << "," <<
              std::to_string(t_opt_basic.get_acceleration_profiles()[1].get_accelerations_segments()[2]) << "]"
              << std::endl;
    std::cout << "Times z: [" <<
              std::to_string(t_opt_basic.get_acceleration_profiles()[2].get_time_durations_segments()[0]) << "," <<
              std::to_string(t_opt_basic.get_acceleration_profiles()[2].get_time_durations_segments()[1]) << "," <<
              std::to_string(t_opt_basic.get_acceleration_profiles()[2].get_time_durations_segments()[2]) << "]" <<
              " / Accelerations z: [" <<
              std::to_string(t_opt_basic.get_acceleration_profiles()[2].get_accelerations_segments()[0]) << "," <<
              std::to_string(t_opt_basic.get_acceleration_profiles()[2].get_accelerations_segments()[1]) << "," <<
              std::to_string(t_opt_basic.get_acceleration_profiles()[2].get_accelerations_segments()[2]) << "]"
              << std::endl;


    std::cout << "____________________________________________" << std::endl;
    std::cout << "Optimal trajectory duration (TOP-UAV-improved): "
              << std::to_string(t_opt_improved.get_time_optimal_trajectory_duration()) << std::endl;
    std::cout << "Times x: [" <<
              std::to_string(t_opt_improved.get_acceleration_profiles()[0].get_time_durations_segments()[0]) << "," <<
              std::to_string(t_opt_improved.get_acceleration_profiles()[0].get_time_durations_segments()[1]) << "," <<
              std::to_string(t_opt_improved.get_acceleration_profiles()[0].get_time_durations_segments()[2]) << "]" <<
              " / Accelerations x: [" <<
              std::to_string(t_opt_improved.get_acceleration_profiles()[0].get_accelerations_segments()[0]) << "," <<
              std::to_string(t_opt_improved.get_acceleration_profiles()[0].get_accelerations_segments()[1]) << "," <<
              std::to_string(t_opt_improved.get_acceleration_profiles()[0].get_accelerations_segments()[2]) << "]"
              << std::endl;

    std::cout << "Times y: [" <<
              std::to_string(t_opt_improved.get_acceleration_profiles()[1].get_time_durations_segments()[0]) << "," <<
              std::to_string(t_opt_improved.get_acceleration_profiles()[1].get_time_durations_segments()[1]) << "," <<
              std::to_string(t_opt_improved.get_acceleration_profiles()[1].get_time_durations_segments()[2]) << "]" <<
              " / Accelerations y: [" <<
              std::to_string(t_opt_improved.get_acceleration_profiles()[1].get_accelerations_segments()[0]) << "," <<
              std::to_string(t_opt_improved.get_acceleration_profiles()[1].get_accelerations_segments()[1]) << "," <<
              std::to_string(t_opt_improved.get_acceleration_profiles()[1].get_accelerations_segments()[2]) << "]"
              << std::endl;
    std::cout << "Times z: [" <<
              std::to_string(t_opt_improved.get_acceleration_profiles()[2].get_time_durations_segments()[0]) << "," <<
              std::to_string(t_opt_improved.get_acceleration_profiles()[2].get_time_durations_segments()[1]) << "," <<
              std::to_string(t_opt_improved.get_acceleration_profiles()[2].get_time_durations_segments()[2]) << "]" <<
              " / Accelerations z: [" <<
              std::to_string(t_opt_improved.get_acceleration_profiles()[2].get_accelerations_segments()[0]) << "," <<
              std::to_string(t_opt_improved.get_acceleration_profiles()[2].get_accelerations_segments()[1]) << "," <<
              std::to_string(t_opt_improved.get_acceleration_profiles()[2].get_accelerations_segments()[2]) << "]"
              << std::endl;
}