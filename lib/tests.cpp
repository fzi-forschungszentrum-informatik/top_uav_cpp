#include "Trajectory_Planner_Single_Axis.h"
#include "Trajectory_Planner.h"

#include <catch2/catch_test_macros.hpp>
#include <catch2/benchmark/catch_benchmark.hpp>

using namespace fzi::top_uav;

TEST_CASE("Trajectory planning single axis: SOTA")
{
    auto sut = Trajectory_Planner_Single_Axis();
    SECTION("Simple case")
    {
        REQUIRE(sut.calc_opt_time(0.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0) == 1.0);
    }
}

TEST_CASE("Trajectory planning multiple axes: TOP-UAV")
{
    SECTION("Example from paper")
    {
        auto top_uav = Trajectory_Planner(4.0, 1.0, "basic");
        REQUIRE(std::round(top_uav.calc_opt_time(0.1, 3.6, 2.0, 0.4, 4.3, 2.6, 0.1, 0.1, -1.9, -1.8, -0.4, 0.6).get_time_optimal_trajectory_duration() * 10000)/10000 == 11.8872);
    }

    SECTION("TOP-UAV++")
    {
        auto top_uav = Trajectory_Planner(4.0, 1.0, "improved");
        REQUIRE(std::round(top_uav.calc_opt_time(0.1, 3.6, 2.0, 0.4, 4.3, 2.6, 0.1, 0.1, -1.9, -1.8, -0.4, 0.6).get_time_optimal_trajectory_duration() * 10000)/10000 == 7.5704);
    }
}

TEST_CASE("Trajectory planning multiple axes: TOP-UAV++")
{
    SECTION("Example from paper")
    {
        auto top_uav = Trajectory_Planner(4.0, 1.0, "improved");
        REQUIRE(std::round(top_uav.calc_opt_time(0.1, 3.6, 2.0, 0.4, 4.3, 2.6, 0.1, 0.1, -1.9, -1.8, -0.4, 0.6).get_time_optimal_trajectory_duration() * 10000)/10000 == 7.5704);
    }
}
