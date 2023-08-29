//
// Created by Fabian Meyer on 2023-02-13.
//

#include "Trajectory_Planner.h"
#include <limits>


fzi::top_uav::Trajectory_Planner::Trajectory_Planner(double v_max, double a_max, std::string version)
        : version(version)
        , traj_planner_single_axis(Trajectory_Planner_Single_Axis())
        , solution(Solution())
        , best_solution(Solution())
{
    if (version == "basic" || version == "sota") {
        configs.emplace_back(v_max / sqrt2(3), v_max / sqrt2(3), v_max / sqrt2(3), a_max / sqrt2(3), a_max / sqrt2(3), a_max / sqrt2(3));
    } else if (version == "improved") {
        configs.emplace_back(v_max / sqrt2(3), v_max / sqrt2(3), v_max / sqrt2(3), a_max / sqrt2(3), a_max / sqrt2(3), a_max / sqrt2(3));  // undistorted
        configs.emplace_back(v_max * sqrt2(3) / 2, v_max / sqrt2(8), v_max / sqrt2(8), a_max * sqrt2(3) / 2, a_max / sqrt2(8), a_max / sqrt2(8));  // distorted towards x
        configs.emplace_back(v_max / sqrt2(8), v_max * sqrt2(3) / 2, v_max / sqrt2(8), a_max / sqrt2(8), a_max * sqrt2(3) / 2, a_max / sqrt2(8));  // distorted towards y
        configs.emplace_back(v_max / sqrt2(8), v_max / sqrt2(8), v_max * sqrt2(3) / 2, a_max / sqrt2(8), a_max / sqrt2(8), a_max * sqrt2(3) / 2);  // distorted towards z
    }
    else {
        throw std::invalid_argument("Wrong input of argument version! Argument must be either basic or improved");
    }
}

const fzi::top_uav::Solution& fzi::top_uav::Trajectory_Planner::calc_opt_time(const double& x_s, const double& x_e, const double& y_s, const double& y_e, const double& z_s, const double& z_e, const double& v_xs, const double& v_xe, const double& v_ys, const double& v_ye, const double& v_zs, const double& v_ze)
{
    double t_opt_best = std::numeric_limits<double>::max();
    for (const auto& config : configs) {
        if (check_inputs(v_xs, v_xe, v_ys, v_ye, v_zs, v_ze, config)) {
            const double& v_max_x = config.get_v_max_x();
            const double& v_min_x = -v_max_x;
            const double& v_max_y = config.get_v_max_y();
            const double& v_min_y = -v_max_y;
            const double& v_max_z = config.get_v_max_z();
            const double& v_min_z = -v_max_z;

            const double& a_max_x = config.get_a_max_x();
            const double& a_min_x = -a_max_x;
            const double& a_max_y = config.get_a_max_y();
            const double& a_min_y = -a_max_y;
            const double& a_max_z = config.get_a_max_z();
            const double& a_min_z = -a_max_z;

            double t_opt_x = traj_planner_single_axis.calc_opt_time(x_s, x_e, v_xs, v_xe, v_min_x, v_max_x, a_min_x, a_max_x);
            double t_opt_y = traj_planner_single_axis.calc_opt_time(y_s, y_e, v_ys, v_ye, v_min_y, v_max_y, a_min_y, a_max_y);
            double t_opt_z = traj_planner_single_axis.calc_opt_time(z_s, z_e, v_zs, v_ze, v_min_z, v_max_z, a_min_z, a_max_z);
            double t_opt = std::max(std::max(t_opt_x, t_opt_y), t_opt_z);
            if (version == "sota")
            {
                solution.set_time_optimal_trajectory_duration(t_opt);
                return solution;
            }

            // check if trajectory can be synchronized with t_opt = t_sota
            if (t_opt < t_opt_best) {
                bool b_synchronization_possible = true;
                b_synchronization_possible &= synchronization_possible(t_opt, x_s, x_e, v_xs, v_xe, v_min_x, v_max_x, a_min_x, a_max_x, 'x');
                b_synchronization_possible &= synchronization_possible(t_opt, y_s, y_e, v_ys, v_ye, v_min_y, v_max_y, a_min_y, a_max_y, 'y');
                b_synchronization_possible &= synchronization_possible(t_opt, z_s, z_e, v_zs, v_ze, v_min_z, v_max_z, a_min_z, a_max_z, 'z');

                if (b_synchronization_possible) {
                        t_opt_best = t_opt;
                        best_solution.copy(solution);
                        best_solution.set_time_optimal_trajectory_duration(t_opt_best);
                }
                else {
                    std::vector<double> t_sync_cand_sorted = determine_candidate_times(t_opt, x_s, x_e, y_s, y_e, z_s, z_e, v_xs, v_xe, v_ys, v_ye, v_zs, v_ze, config);
                    for (const auto& elem : t_sync_cand_sorted) {
                        if (elem < t_opt_best) {
                            b_synchronization_possible = true;
                            b_synchronization_possible &= synchronization_possible(elem, x_s, x_e, v_xs, v_xe, v_min_x, v_max_x, a_min_x, a_max_x, 'x');
                            b_synchronization_possible &= synchronization_possible(elem, y_s, y_e, v_ys, v_ye, v_min_y, v_max_y, a_min_y, a_max_y, 'y');
                            b_synchronization_possible &= synchronization_possible(elem, z_s, z_e, v_zs, v_ze, v_min_z, v_max_z, a_min_z, a_max_z, 'z');
                            if (b_synchronization_possible) {

                                    t_opt_best = elem;
                                    best_solution.copy(solution);
                                    best_solution.set_time_optimal_trajectory_duration(t_opt_best);
                                    break;
                            }
                        }
                    }
                }

            }

        }
    }
    return best_solution;
}

bool fzi::top_uav::Trajectory_Planner::synchronization_possible(const double& t_sync, const double& p_s, const double& p_e, const double& v_s, const double& v_e, const double& v_min, const double& v_max, const double& a_min, const double& a_max, const char& axis)
{
    bool sync_valid = false;
    sync_valid |= sync_possible_pattern1(t_sync, p_s, p_e, v_s, v_e, v_min, v_max, a_min, a_max, axis);
    if (sync_valid) { return true; };
    sync_valid |= sync_possible_pattern2(t_sync, p_s, p_e, v_s, v_e, v_min, v_max, a_min, a_max, axis);
    if (sync_valid) { return true; };
    sync_valid |= sync_possible_pattern3(t_sync, p_s, p_e, v_s, v_e, v_min, v_max, a_min, a_max, axis);
    if (sync_valid) { return true; };
    sync_valid |= sync_possible_pattern4(t_sync, p_s, p_e, v_s, v_e, v_min, v_max, a_min, a_max, axis);
    if (sync_valid) { return true; };

    return false;
}

bool fzi::top_uav::Trajectory_Planner::check_inputs(const double& v_xs, const double& v_xe, const double& v_ys, const double& v_ye, const double& v_zs, const double& v_ze, const Config& config)
{
    const double& v_max_x = config.get_v_max_x();
    const double& v_max_y = config.get_v_max_y();
    const double& v_max_z = config.get_v_max_z();

    if (v_xs < -v_max_x) {
        return false;
    }
    if (v_xs > v_max_x) {
        return false;
    }
    if (v_xe < -v_max_x) {
        return false;
    }
    if (v_xe > v_max_x) {
        return false;
    }
    if (v_ys < -v_max_y) {
        return false;
    }
    if (v_ys > v_max_y) {
        return false;
    }
    if (v_ye < -v_max_y) {
        return false;
    }
    if (v_ye > v_max_y) {
        return false;
    }
    if (v_zs < -v_max_z) {
        return false;
    }
    if (v_zs > v_max_z) {
        return false;
    }
    if (v_ze < -v_max_z) {
        return false;
    }
    if (v_ze > v_max_z) {
        return false;
    }
    return true;
}

bool fzi::top_uav::Trajectory_Planner::sync_possible_pattern1(const double& t_sync, const double& p_s, const double& p_e, const double& v_s, const double& v_e, const double& v_min, const double& v_max, const double& a_min, const double& a_max, const char& axis)
{
    ////////////////////////////
    // PATTERN 1: (-a, 0, +a) //
    ////////////////////////////
    const double& a = a_min;

    const double A = round_customized(pow2(a) * pow2(t_sync) + 2 * (v_e + v_s) * a * t_sync - 4 * p_e * a + 4 * p_s * a - pow2((v_e - v_s)));
    if (A >= 0) {
        const double sqrt_A = sqrt2(A);
        const double t1 = (a * t_sync + v_e - v_s + sqrt_A) / (2 * a);
        const double t2 = (-sqrt_A) / a;
        const double t3 = (a * t_sync - v_e + v_s + sqrt_A) / (2 * a);
        const double v_k = (a * t_sync + v_e + v_s + sqrt_A) / 2;

        if (t1 > -0.0001 && t2 > -0.0001 && t3 > -0.0001 && v_k - v_min > -0.0001 && v_k - v_max < 0.0001) {
            solution.set_acceleration_profile(a_min, 0.0, a_max, t1, t2, t3, axis);

            return true;
        }
    }
    return false;
}

bool fzi::top_uav::Trajectory_Planner::sync_possible_pattern2(const double& t_sync, const double& p_s, const double& p_e, const double& v_s, const double& v_e, const double& v_min, const double& v_max, const double& a_min, const double& a_max, const char& axis)
{
    ////////////////////////////
    // PATTERN 2: (+a, 0, -a) //
    ////////////////////////////
    const double& a = a_max;

    const double  A = round_customized(pow2(a) * pow2(t_sync) + 2 * (v_e + v_s) * a * t_sync - 4 * p_e * a + 4 * p_s * a - pow2((v_e - v_s)));
    if (A >= 0) {
        const double sqrt_A = sqrt2(A);
        const double t1 = (a * t_sync + v_e - v_s - sqrt_A) / (2 * a);
        const double t2 = sqrt_A / a;
        const double t3 = (a * t_sync - v_e + v_s - sqrt_A) / (2 * a);
        const double v_k = (a * t_sync + v_e + v_s - sqrt_A) / 2;

        if (t1 > -0.0001 && t2 > -0.0001 && t3 > -0.0001 && v_k - v_min > -0.0001 && v_k - v_max < 0.0001) {
            solution.set_acceleration_profile(a_max, 0.0, a_min, t1, t2, t3, axis);
            return true;
        }
    }
    return false;
}

bool fzi::top_uav::Trajectory_Planner::sync_possible_pattern3(const double& t_sync, const double& p_s, const double& p_e, const double& v_s, const double& v_e, const double& v_min, const double& v_max, const double& a_min, const double& a_max, const char& axis)
{
    ////////////////////////////
    // PATTERN 3: (+a, 0, +a) //
    ////////////////////////////
    const double& a = a_max;

    if (t_sync * a - v_e + v_s != 0) {
        const double t1 = ((-2 * v_s * t_sync + 2 * p_e - 2 * p_s) * a - pow2((v_e - v_s))) / (2 * a * (t_sync * a - v_e + v_s));
        const double t2 = (t_sync * a - v_e + v_s) / a;
        const double t3 = ((2 * v_e * t_sync - 2 * p_e + 2 * p_s) * a - pow2((v_e - v_s))) / (2 * a * (t_sync * a - v_e + v_s));
        const double v_k = v_s + a * t1;

        if (t1 > -0.0001 && t2 > -0.0001 && t3 > -0.0001 && v_k - v_min > -0.0001 && v_k - v_max < 0.0001) {
            solution.set_acceleration_profile(a_max, 0.0, a_max, t1, t2, t3, axis);
            return true;
        }
    }
    return false;
}

bool fzi::top_uav::Trajectory_Planner::sync_possible_pattern4(const double& t_sync, const double& p_s, const double& p_e, const double& v_s, const double& v_e, const double& v_min, const double& v_max, const double& a_min, const double& a_max, const char& axis)
{
    ////////////////////////////
    // PATTERN 4: (-a, 0, -a) //
    ////////////////////////////
    const double& a = a_min;

    if (t_sync * a - v_e + v_s != 0) {
        const double t1 = ((-2 * v_s * t_sync + 2 * p_e - 2 * p_s) * a - pow2((v_e - v_s))) / (2 * a * (t_sync * a - v_e + v_s));
        const double t2 = (t_sync * a - v_e + v_s) / a;
        const double t3 = ((2 * v_e * t_sync - 2 * p_e + 2 * p_s) * a - pow2((v_e - v_s))) / (2 * a * (t_sync * a - v_e + v_s));
        const double v_k = v_s + a * t1;

        if (t1 > -0.0001 && t2 > -0.0001 && t3 > -0.0001 && v_k - v_min > -0.0001 && v_k - v_max < 0.0001) {
            solution.set_acceleration_profile(a_min, 0.0, a_min, t1, t2, t3, axis);
            return true;
        }
    }
    return false;
}

std::vector<double> fzi::top_uav::Trajectory_Planner::determine_candidate_times(const double& t_opt, const double& x_s, const double& x_e, const double& y_s, const double& y_e, const double& z_s, const double& z_e, const double& v_xs, const double& v_xe, const double& v_ys, const double& v_ye, const double& v_zs, const double& v_ze, const Config& config)
{
    const double& v_max_x = config.get_v_max_x();
    const double& v_min_x = -v_max_x;
    const double& v_max_y = config.get_v_max_y();
    const double& v_min_y = -v_max_y;
    const double& v_max_z = config.get_v_max_z();
    const double& v_min_z = -v_max_z;

    const double& a_max_x = config.get_a_max_x();
    const double& a_min_x = -a_max_x;
    const double& a_max_y = config.get_a_max_y();
    const double& a_min_y = -a_max_y;
    const double& a_max_z = config.get_a_max_z();
    const double& a_min_z = -a_max_z;
    std::vector<double> candidates;

    // x-axis
    sync_pattern1(x_s, x_e, v_xs, v_xe, v_max_x, a_max_x, candidates);
    sync_pattern2(x_s, x_e, v_xs, v_xe, v_min_x, a_min_x, candidates);
    sync_pattern3(x_s, x_e, v_xs, v_xe, a_max_x, candidates);
    sync_pattern4(x_s, x_e, v_xs, v_xe, a_min_x, candidates);


    //y-axis
    sync_pattern1(y_s, y_e, v_ys, v_ye, v_max_y, a_max_y, candidates);
    sync_pattern2(y_s, y_e, v_ys, v_ye, v_min_y, a_min_y, candidates);
    sync_pattern3(y_s, y_e, v_ys, v_ye, a_max_y, candidates);
    sync_pattern4(y_s, y_e, v_ys, v_ye, a_min_y, candidates);


    //z-axis
    sync_pattern1(z_s, z_e, v_zs, v_ze, v_max_z, a_max_z, candidates);
    sync_pattern2(z_s, z_e, v_zs, v_ze, v_min_z, a_min_z, candidates);
    sync_pattern3(z_s, z_e, v_zs, v_ze, a_max_z, candidates);
    sync_pattern4(z_s, z_e, v_zs, v_ze, a_min_z, candidates);


    std::vector<double> candidates_purged;
    for (const auto& value : candidates) {
        if (value > t_opt) {		// value == t_opt already checked beforehand
            candidates_purged.push_back(value);
        }
    }

    // sort and remove duplicates
    std::sort(candidates_purged.begin(), candidates_purged.end());
    candidates_purged.erase(std::unique(candidates_purged.begin(), candidates_purged.end()), candidates_purged.end());

    return candidates_purged;
}

void fzi::top_uav::Trajectory_Planner::sync_pattern1(const double& p_s, const double& p_e, const double& v_s, const double& v_e, const double& v_max, const double& a_max, std::vector<double>& candidates)
{
    ////////////////////////////
    // PATTERN 1: (+a, 0, -a) //
    ////////////////////////////
    double a = a_max;

    // zero of equation t1(te) == 0
    if (v_s == 0) {
        candidates.push_back(0);
    }
    else {
        candidates.push_back((pow2(v_s) - 2 * v_e * v_s + (2 * p_e - 2 * p_s) * a + pow2(v_e)) / (2 * a * v_s));
    }

    // zero of equation t2(te) == 0
    double value = 4 * a * p_e - 4 * a * p_s + 2 * pow2(v_e) + 2 * pow2(v_s);
    if (value > 0) {
        double sqrt_val = sqrt2(4 * a * p_e - 4 * a * p_s + 2 * pow2(v_e) + 2 * pow2(v_s));
        candidates.push_back((-v_e - v_s + sqrt_val) / a);
        candidates.push_back((-v_e - v_s - sqrt_val) / a);
    }

    else {
        candidates.push_back(std::numeric_limits<double>::min());
        candidates.push_back(std::numeric_limits<double>::min());
    }

    // zero of equation t3(te) == 0
    if (v_e == 0) {
        candidates.push_back(0);
    }
    else {
        candidates.push_back((2 * a * p_e - 2 * a * p_s + pow2(v_e) - 2 * v_e * v_s + pow2(v_s)) / (2 * a * v_e));
    }

    // zeros of equation v_k(te) == v_max
    candidates.push_back((2 * a * p_e - 2 * a * p_s + pow2(v_e) - 2 * v_e * v_max + 2 * pow2(v_max) - 2 * v_max * v_s + pow2(v_s)) / (2 * a * v_max));
}

void fzi::top_uav::Trajectory_Planner::sync_pattern2(const double& p_s, const double& p_e, const double& v_s, const double& v_e, const double& v_min, const double& a_min, std::vector<double>& candidates)
{
    ////////////////////////////
    // PATTERN 2: (-a, 0, +a) //
    ////////////////////////////
    double a = a_min;

    // zero of equation t1(te) == 0
    if (v_s == 0) {
        candidates.push_back(0);
    }
    else {
        candidates.push_back((pow2(v_s) - 2 * v_e * v_s + (2 * p_e - 2 * p_s) * a + pow2(v_e)) / (2 * a * v_s));
    }

    // zero of equation t2(te) == 0
    double value = 4 * a * p_e - 4 * a * p_s + 2 * pow2(v_e) + 2 * pow2(v_s);
    if (value > 0) {
        double sqrt_val = sqrt2(4 * a * p_e - 4 * a * p_s + 2 * pow2(v_e) + 2 * pow2(v_s));
        candidates.push_back((-v_e - v_s + sqrt_val) / a);
        candidates.push_back((-v_e - v_s - sqrt_val) / a);

    }
    else if (value == 0)
    {
        candidates.push_back((-v_e - v_s) / a);
    }
    else {
        candidates.push_back(std::numeric_limits<double>::min());
        candidates.push_back(std::numeric_limits<double>::min());
    }

    // zero of equation t3(te) == 0
    if (v_e == 0) {
        candidates.push_back(0);
    }
    else {
        candidates.push_back((2 * a * p_e - 2 * a * p_s + pow2(v_e) - 2 * v_e * v_s + pow2(v_s)) / (2 * a * v_e));
    }

    // zero of equation v_k(te) == v_max
    candidates.push_back((2 * a * p_e - 2 * a * p_s + pow2(v_e) - 2 * v_e * v_min + 2 * pow2(v_min) - 2 * v_min * v_s + pow2(v_s)) / (2 * a * v_min));
}

void fzi::top_uav::Trajectory_Planner::sync_pattern3(const double& p_s, const double& p_e, const double& v_s, const double& v_e, const double& a_max, std::vector<double>& candidates)
{
    ////////////////////////////
    // PATTERN 3: (+a, 0, +a) //
    ////////////////////////////
    double a = a_max;

    // zero of equation t1(te) == 0
    if (v_s == 0) {
        candidates.push_back(0);
    }
    else {
        candidates.push_back((2 * a * p_e - 2 * a * p_s - pow2(v_e) + 2 * v_e * v_s - pow2(v_s)) / (2 * a * v_s));
    }

    // zero of equation t2(te) == 0
    candidates.push_back((v_e - v_s) / a);

    // zero of equation t3(te) == 0
    if (v_e == 0) {
        candidates.push_back(0);
    }
    else {
        candidates.push_back((2 * a * p_e - 2 * a * p_s + pow2(v_e) - 2 * v_e * v_s + pow2(v_s)) / (2 * a * v_e));
    }
}

void fzi::top_uav::Trajectory_Planner::sync_pattern4(const double& p_s, const double& p_e, const double& v_s, const double& v_e, const double& a_min, std::vector<double>& candidates)
{
    ////////////////////////////
    // PATTERN 4: (-a, 0, -a) //
    ////////////////////////////
    double a = a_min;

    // zero of equation t1(te) == 0
    if (v_s == 0) {
        candidates.push_back(0);
    }
    else {
        candidates.push_back((2 * a * p_e - 2 * a * p_s - pow2(v_e) + 2 * v_e * v_s - pow2(v_s)) / (2 * a * v_s));
    }

    // zero of equation t2(te) == 0
    candidates.push_back((v_e - v_s) / a);

    // zero of equation t3(te) == 0
    if (v_e == 0) {
        candidates.push_back(0);
    }
    else {
        candidates.push_back((2 * a * p_e - 2 * a * p_s + pow2(v_e) - 2 * v_e * v_s + pow2(v_s)) / (2 * a * v_e));
    }
}
