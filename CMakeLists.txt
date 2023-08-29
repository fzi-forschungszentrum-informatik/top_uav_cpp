cmake_minimum_required(VERSION 3.22)
project(top_uav_cpp)

set(CMAKE_CXX_STANDARD 17)
add_subdirectory(lib)

add_executable(top_uav_cpp main.cpp)
target_link_libraries(top_uav_cpp PRIVATE lib)
