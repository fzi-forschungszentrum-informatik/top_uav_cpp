cmake_minimum_required(VERSION 3.22)
project(trajectory_generation_lib)

set(CMAKE_CXX_STANDARD 17)

add_library(lib
        Acceleration_Profile.cpp
        Acceleration_Profile.h
        Config.cpp
        Config.h
        Solution.cpp
        Solution.h
        Trajectory_Planner.cpp
        Trajectory_Planner.h
        Trajectory_Planner_Single_Axis.cpp
        Trajectory_Planner_Single_Axis.h
        utils.h)

target_include_directories(lib PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")

add_catch2_test_executable(tests SOURCES tests.cpp DEPENDS lib)
