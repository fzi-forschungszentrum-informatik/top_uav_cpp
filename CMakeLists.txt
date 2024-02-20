cmake_minimum_required(VERSION 3.22)
project(top_uav_cpp)

set(CMAKE_CXX_STANDARD 17)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/cmake)
include(Catch2Integration)

add_subdirectory(lib)

add_executable(top_uav_cpp main.cpp)
target_link_libraries(top_uav_cpp PRIVATE lib)

add_executable(benchmarks benchmarks.cpp)
target_link_libraries(benchmarks
  PRIVATE Catch2::Catch2WithMain lib
)
