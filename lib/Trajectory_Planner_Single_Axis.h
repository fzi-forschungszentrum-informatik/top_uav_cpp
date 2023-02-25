//
// Created by Fabian Meyer on 2023-02-13.
//
#pragma once
#ifndef TRAJECTORY_GENERATION_LIB_TRAJECTORY_PLANNER_SINGLE_AXIS_H
#define TRAJECTORY_GENERATION_LIB_TRAJECTORY_PLANNER_SINGLE_AXIS_H

#include "utils.h"
#include <cmath>
#include <algorithm>



namespace fzi {
    namespace top_uav {

        class Trajectory_Planner_Single_Axis
        {
        public:
            Trajectory_Planner_Single_Axis() = default;
            double calc_opt_time(const double& p_s, const double& p_e, const double& v_s, const double& v_e, const double& v_min, const double& v_max, const double& a_min, const double& a_max);

        private:
            /*double v_min, v_max, a_min, a_max;*/
            double case1(const double& p_s, const double& p_e, const double& v_s, const double& v_e, const double& v_min, const double& v_max, const double& a_min, const double& a_max);
            double case2(const double& p_s, const double& p_e, const double& v_s, const double& v_e, const double& v_min, const double& v_max, const double& a_min, const double& a_max);
            double case3(const double& p_s, const double& p_e, const double& v_s, const double& v_e, const double& v_min, const double& v_max, const double& a_min, const double& a_max);
            double case4(const double& p_s, const double& p_e, const double& v_s, const double& v_e, const double& v_min, const double& v_max, const double& a_min, const double& a_max);

        };

    } // fzi
} // top_uav



#endif //TRAJECTORY_GENERATION_LIB_TRAJECTORY_PLANNER_SINGLE_AXIS_H
