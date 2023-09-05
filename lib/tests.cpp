#include "Trajectory_Planner_Single_Axis.h"

#include <catch2/catch_test_macros.hpp>
#include <catch2/benchmark/catch_benchmark.hpp>

using namespace fzi::top_uav;

TEST_CASE("Trajectory single axis")
{
    auto sut = Trajectory_Planner_Single_Axis();
    SECTION("Simple case")
    {
        REQUIRE(sut.calc_opt_time(0.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0) == 1.0);
    }
    // SECTION( "Zero acceleration" ) {
    //     REQUIRE(sut.calc_opt_time(0.0,1.0,1.0,1.0,1.0,1.0,0,0) == 1.0);
    // }
}
