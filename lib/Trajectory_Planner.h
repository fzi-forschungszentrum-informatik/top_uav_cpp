//
// Created by Fabian Meyer on 2023-02-13.
//

#pragma once
#ifndef TRAJECTORY_GENERATION_LIB_TRAJECTORY_PLANNER_H
#define TRAJECTORY_GENERATION_LIB_TRAJECTORY_PLANNER_H
#include "Trajectory_Planner_Single_Axis.h"
#include "Config.h"
#include "Solution.h"
#include "utils.h"
#include <vector>
#include <string>
#include <algorithm>
#include <stdexcept>

namespace fzi {
    namespace top_uav {

        class Trajectory_Planner
        {
        public:
            Trajectory_Planner(double v_max, double a_max, std::string version);
            const Solution& calc_opt_time(const double& x_s, const double& x_e, const double& y_s, const double& y_e, const double& z_s, const double& z_e, const double& v_xs, const double& v_xe, const double& v_ys, const double& v_ye, const double& v_zs, const double& v_ze);
            bool synchronization_possible(const double& t_opt, const double& p_s, const double& p_e, const double& v_s, const double& v_e, const double& v_min, const double& v_max, const double& a_min, const double& a_max, const char& axis);

        private:
            Solution solution, best_solution;
            Configs configs;
            std::string version;
            Trajectory_Planner_Single_Axis traj_planner_single_axis;
            bool check_inputs(const double& v_xs, const double& v_xe, const double& v_ys, const double& v_ye, const double& v_zs, const double& v_ze, const Config& config);

            bool sync_possible_pattern1(const double& t_sync, const double& p_s, const double& p_e, const double& v_s, const double& v_e, const double& v_min, const double& v_max, const double& a_min, const double& a_max, const char& axis);
            bool sync_possible_pattern2(const double& t_sync, const double& p_s, const double& p_e, const double& v_s, const double& v_e, const double& v_min, const double& v_max, const double& a_min, const double& a_max, const char& axis);
            bool sync_possible_pattern3(const double& t_sync, const double& p_s, const double& p_e, const double& v_s, const double& v_e, const double& v_min, const double& v_max, const double& a_min, const double& a_max, const char& axis);
            bool sync_possible_pattern4(const double& t_sync, const double& p_s, const double& p_e, const double& v_s, const double& v_e, const double& v_min, const double& v_max, const double& a_min, const double& a_max, const char& axis);

            std::vector<double> determine_candidate_times(const double& t_opt, const double& x_s, const double& x_e, const double& y_s, const double& y_e, const double& z_s, const double& z_e, const double& v_xs, const double& v_xe, const double& v_ys, const double& v_ye, const double& v_zs, const double& v_ze, const Config& config);

            void sync_pattern1(const double& p_s, const double& p_e, const double& v_s, const double& v_e, const double& v_max, const double& a_max, std::vector<double>& candidates);
            void sync_pattern2(const double& p_s, const double& p_e, const double& v_s, const double& v_e, const double& v_min, const double& a_min, std::vector<double>& candidates);
            void sync_pattern3(const double& p_s, const double& p_e, const double& v_s, const double& v_e, const double& a_max, std::vector<double>& candidates);
            void sync_pattern4(const double& p_s, const double& p_e, const double& v_s, const double& v_e, const double& a_min, std::vector<double>& candidates);
        };

    } // fzi
} // top_uav

#endif //TRAJECTORY_GENERATION_LIB_TRAJECTORY_PLANNER_H
