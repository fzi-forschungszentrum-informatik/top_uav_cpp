//
// Created by Fabian Meyer on 2023-02-13.
//

#include "Config.h"

fzi::top_uav::Config::Config(double v_max_x, double v_max_y, double v_max_z, double a_max_x, double a_max_y, double a_max_z)
        : v_max_x(v_max_x)
        , v_max_y(v_max_y)
        , v_max_z(v_max_z)
        , a_max_x(a_max_x)
        , a_max_y(a_max_y)
        , a_max_z(a_max_z)
{
}
