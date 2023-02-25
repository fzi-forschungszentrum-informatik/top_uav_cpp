//
// Created by Fabian Meyer on 2023-02-13.
//

#ifndef TRAJECTORY_GENERATION_LIB_CONFIG_H
#define TRAJECTORY_GENERATION_LIB_CONFIG_H
#include<vector>

namespace fzi {
    namespace top_uav {

        class Config
        {
        public:
            Config(double v_max_x, double v_max_y, double v_max_z, double a_max_x, double a_max_y, double a_max_z);
            const double& get_v_max_x() const { return v_max_x; };
            const double& get_v_max_y() const { return v_max_y; };
            const double& get_v_max_z() const { return v_max_z; };
            const double& get_a_max_x() const { return a_max_x; };
            const double& get_a_max_y() const { return a_max_y; };
            const double& get_a_max_z() const { return a_max_z; };
        private:
            double v_max_x, v_max_y, v_max_z, a_max_x, a_max_y, a_max_z;
        };

        typedef std::vector<Config> Configs;
    } // fzi
} // top_uav

#endif //TRAJECTORY_GENERATION_LIB_CONFIG_H
