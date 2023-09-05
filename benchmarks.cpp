#include "Trajectory_Planner.h"

#include <catch2/catch_test_macros.hpp>
#include <catch2/benchmark/catch_benchmark.hpp>

using namespace fzi::top_uav;

TEST_CASE("Calculate opt_time for example")
{
    double v_max = 4.0;
    double a_max = 1.0;

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

    Trajectory_Planner sota(v_max, a_max, "sota");
    BENCHMARK("example opt_sota")
    {
        return sota.calc_opt_time(px_s, px_e, py_s, py_e, pz_s, pz_e,
                                  vx_s, vx_e, vy_s, vy_e, vz_s, vz_e);
    };

    Trajectory_Planner basic(v_max, a_max, "basic");
    BENCHMARK("example basic")
    {
        return basic.calc_opt_time(px_s, px_e, py_s, py_e, pz_s, pz_e,
                                   vx_s, vx_e, vy_s, vy_e, vz_s, vz_e);
    };

    Trajectory_Planner improved(v_max, a_max, "improved");
    BENCHMARK("example improved")
    {
        return improved.calc_opt_time(px_s, px_e, py_s, py_e, pz_s, pz_e,
                                      vx_s, vx_e, vy_s, vy_e, vz_s, vz_e);
    };
}
