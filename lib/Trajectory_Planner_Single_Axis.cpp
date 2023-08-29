//
// Created by Fabian Meyer on 2023-02-13.
//

#include "Trajectory_Planner_Single_Axis.h"
#include <limits>


double fzi::top_uav::Trajectory_Planner_Single_Axis::calc_opt_time(const double& p_s, const double& p_e, const double& v_s, const double& v_e, const double& v_min, const double& v_max, const double& a_min, const double& a_max)
{
    double t_opt = std::numeric_limits<double>::max();

    double t_case1 = case1(p_s, p_e, v_s, v_e, v_min, v_max, a_min, a_max);
    double t_case2 = case2(p_s, p_e, v_s, v_e, v_min, v_max, a_min, a_max);
    double t_case3 = case3(p_s, p_e, v_s, v_e, v_min, v_max, a_min, a_max);
    double t_case4 = case4(p_s, p_e, v_s, v_e, v_min, v_max, a_min, a_max);

    t_opt = std::min({ t_case1, t_case2, t_case3, t_case4 });

    return t_opt;
}

double fzi::top_uav::Trajectory_Planner_Single_Axis::case1(const double& p_s, const double& p_e, const double& v_s, const double& v_e, const double& v_min, const double& v_max, const double& a_min, const double& a_max)
{
    ////////////////////////////
    // CASE1: (+a, 0, -a) //////
    // VELOCITY LIMIT REACHED //
    ////////////////////////////

    bool valid = true;

    double t_tot = std::numeric_limits<double>::max();
    const double t1 = (v_max - v_s) / a_max;
    const double t2 = 0.5 * (2 * p_e * a_max * a_min - 2 * p_s * a_max * a_min - a_max * pow2(v_e) + a_max * pow2(v_max) - pow2(v_max) * a_min + pow2(v_s) * a_min) / a_max / a_min / v_max;
    const double t3 = (v_e - v_max) / a_min;

    double t_seg[3] = { t1, t2, t3 };

    for (int i = 0; i < 3; i++) {
        if (t_seg[i] < -0.000001) {
            // valid = false;
            return t_tot;
        }
    }

    const double v1 = v_s;
    const double v2 = v_max;
    const double v3 = v_max;
    const double v4 = v_e;

    const double v_array[4] = { v1, v2, v3, v4 };

    for (const double& v_elem : v_array) {
        if (v_elem < v_min - 0.000001 || v_elem > v_max + 0.000001) {
            // valid = false;
            return t_tot;
        }
    }

    if (valid) {
        t_tot = t1 + t2 + t3;
    }

    return t_tot;
}

double fzi::top_uav::Trajectory_Planner_Single_Axis::case2(const double& p_s, const double& p_e, const double& v_s, const double& v_e, const double& v_min, const double& v_max, const double& a_min, const double& a_max)
{
    ////////////////////////////
    // CASE2: (-a, 0, +a) //////
    // VELOCITY LIMIT REACHED //
    ////////////////////////////

    bool valid = true;

    double t_tot = std::numeric_limits<double>::max();
    const double t1 = (v_min - v_s) / a_min;
    const double t2 = 0.5 * (2 * p_e * a_max * a_min - 2 * p_s * a_max * a_min - pow2(v_min) * a_max + pow2(v_s) * a_max - a_min * pow2(v_e) + a_min * pow2(v_min)) / a_max / a_min / v_min;
    const double t3 = (v_e - v_min) / a_max;

    double t_seg[3] = { t1, t2, t3 };

    for (int i = 0; i < 3; i++) {
        if (t_seg[i] < -0.000001) {
            // valid = false;
            return t_tot;
        }
    }

    const double v1 = v_s;
    const double v2 = v_min;
    const double v3 = v_min;
    const double v4 = v_e;

    const double v_array[4] = { v1, v2, v3, v4 };

    for (const double& v_elem : v_array) {
        if (v_elem < v_min - 0.000001 || v_elem > v_max + 0.000001) {
            // valid = false;
            return t_tot;
        }
    }

    if (valid) {
        t_tot = t1 + t2 + t3;
    }

    return t_tot;
}

double fzi::top_uav::Trajectory_Planner_Single_Axis::case3(const double& p_s, const double& p_e, const double& v_s, const double& v_e, const double& v_min, const double& v_max, const double& a_min, const double& a_max)
{
    ////////////////////////////////
    // CASE3: (+a, -a) /////////////
    // VELOCITY LIMIT NOT REACHED //
    ////////////////////////////////

    bool valid = true;

    double t_tot = std::numeric_limits<double>::max();
    const double t1 = -((a_max * v_e - a_min * v_e - sqrt2(-2 * pow2(a_max) * a_min * p_e + 2 * pow2(a_max) * a_min * p_s + pow2(a_max) * pow2(v_e) + 2 * a_max * pow2(a_min) * p_e - 2 * a_max * pow2(a_min) * p_s - a_max * a_min * pow2(v_e) - a_max * a_min * pow2(v_s) + pow2(a_min) * pow2(v_s))) / (a_max - a_min) - v_e + v_s) / a_max;
    const double t2 = 0;
    const double t3 = (a_max * v_e - a_min * v_e - sqrt2(-2 * pow2(a_max) * a_min * p_e + 2 * pow2(a_max) * a_min * p_s + pow2(a_max) * pow2(v_e) + 2 * a_max * pow2(a_min) * p_e - 2 * a_max * pow2(a_min) * p_s - a_max * a_min * pow2(v_e) - a_max * a_min * pow2(v_s) + pow2(a_min) * pow2(v_s))) / a_min / (a_max - a_min);

    double t_seg[3] = { t1, t2, t3 };

    for (int i = 0; i < 3; i++) {
        if (t_seg[i] < -0.000001) {
            // valid = false;
            return t_tot;
        }
    }

    const double v1 = v_s;
    const double v2 = -(a_max * v_e - a_min * v_e + sqrt2(-2 * pow2(a_max) * a_min * p_e + 2 * pow2(a_max) * a_min * p_s + pow2(a_max) * pow2(v_e) + 2 * a_max * pow2(a_min) * p_e - 2 * a_max * pow2(a_min) * p_s - a_max * a_min * pow2(v_e) - a_max * a_min * pow2(v_s) + pow2(a_min) * pow2(v_s))) / (a_max - a_min) + v_e;
    const double v3 = v2;
    const double v4 = v_e;

    const double v_array[4] = { v1, v2, v3, v4 };

    for (const double& v_elem : v_array) {
        if (v_elem< v_min - 0.000001 || v_elem> v_max + 0.000001) {
            // valid = false;
            return t_tot;
        }
    }

    if (valid) {
        t_tot = t1 + t2 + t3;
    }

    return t_tot;
}

double fzi::top_uav::Trajectory_Planner_Single_Axis::case4(const double& p_s, const double& p_e, const double& v_s, const double& v_e, const double& v_min, const double& v_max, const double& a_min, const double& a_max)
{
    ////////////////////////////////
    // CASE4: (-a, +a) /////////////
    // VELOCITY LIMIT NOT REACHED //
    ////////////////////////////////

    bool valid = true;

    double t_tot = std::numeric_limits<double>::max();
    const double t1 = -((a_max * v_e - a_min * v_e + sqrt2(2 * pow2(a_max) * a_min * p_e - 2 * pow2(a_max) * a_min * p_s + pow2(a_max) * pow2(v_s) - 2 * a_max * pow2(a_min) * p_e + 2 * a_max * pow2(a_min) * p_s - a_max * a_min * pow2(v_e) - a_max * a_min * pow2(v_s) + pow2(a_min) * pow2(v_e))) / (a_max - a_min) - v_e + v_s) / a_min;
    const double t2 = 0;
    const double t3 = (a_max * v_e - a_min * v_e + sqrt2(2 * pow2(a_max) * a_min * p_e - 2 * pow2(a_max) * a_min * p_s + pow2(a_max) * pow2(v_s) - 2 * a_max * pow2(a_min) * p_e + 2 * a_max * pow2(a_min) * p_s - a_max * a_min * pow2(v_e) - a_max * a_min * pow2(v_s) + pow2(a_min) * pow2(v_e))) / a_max / (a_max - a_min);

    double t_seg[3] = { t1, t2, t3 };

    for (int i = 0; i < 3; i++) {
        if (t_seg[i] < -0.000001) {
            // valid = false;
            return t_tot;
        }
    }

    const double v1 = v_s;
    const double v2 = -(a_max * v_e - a_min * v_e + sqrt2(2 * pow2(a_max) * a_min * p_e - 2 * pow2(a_max) * a_min * p_s + pow2(a_max) * pow2(v_s) - 2 * a_max * pow2(a_min) * p_e + 2 * a_max * pow2(a_min) * p_s - a_max * a_min * pow2(v_e) - a_max * a_min * pow2(v_s) + pow2(a_min) * pow2(v_e))) / (a_max - a_min) + v_e;
    const double v3 = v2;
    const double v4 = v_e;

    const double v_array[4] = { v1, v2, v3, v4 };

    for (const double& v_elem: v_array) {
        if (v_elem < v_min - 0.000001 || v_elem > v_max + 0.000001) {
            // valid = false;
            return t_tot;
        }
    }

    if (valid) {
        t_tot = t1 + t2 + t3;
    }

    return t_tot;
}

