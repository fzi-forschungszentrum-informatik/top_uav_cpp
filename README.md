# TOP-UAV: Open-Source Time-optimal Trajectory Planner for Point-Masses under Acceleration and Velocity Constraints (C++)
top_uav_cpp is a C++ tool to generate 3-dimensional time-optimal trajectories from an initial position and velocity vector to a final position and velocity vector with constraints on the maximum velocity and acceleration for the entire motion.

To access the Python implementation see [top_uav_py](https://github.com/fzi-forschungszentrum-informatik/top_uav_py).

## 💈 Build
```shell
mkdir build
cd build
cmake ../
cmake --build . --target top_uav_cpp
```


## 🍫 Quickstart
The example in main.cpp generates time-optimal trajectories according to the state-of-the-art method as well as of our basic generally valid version as well as our version with improved exploitation of kinematic properties.


## ✅ Build and run tests
```shell
# in build directory
cmake --build . --target all_tests
ctest
```


## 🚀 Benchmarks

Use `Release` configuration. Then build the `benchmarks` target and run the executable.


## 🏫 Affiliations
<p align="center">
    <img src="https://upload.wikimedia.org/wikipedia/de/thumb/4/44/Fzi_logo.svg/1200px-Fzi_logo.svg.png?raw=true" alt="FZI Logo" height="200">
</p>

## Citation

If you use top_uav_cpp in your research, please consider citing our original paper. 

```
@INPROCEEDINGS{10342270,
  author={Meyer, Fabian and Glock, Katharina and Sayah, David},
  booktitle={2023 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)}, 
  title={TOP-UAV: Open-Source Time-Optimal Trajectory Planner for Point-Masses Under Acceleration and Velocity Constraints}, 
  year={2023},
  volume={},
  number={},
  pages={2838-2845},
  doi={10.1109/IROS55552.2023.10342270}}
```
