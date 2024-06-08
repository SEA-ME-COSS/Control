# About

This repository is for the Control part of an Autonomous Driving System. It subscribes to paths, current pose and velocities via ROS2, and publishes the vehicle's throttle and steering values. For lateral control, it uses the pure pursuit and stanley algorithms, while for longitudinal control, it employs a PID algorithm. If you're interested in a detailed usage of these algorithms, visit the following repository to test them out.

# Requirements

- Ubuntu 20.04
- [ROS2 foxy](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html)
- matplotlib & numpy
    
    ```bash
    pip install numpy
    pip install matplotlib
    ```
    
- Eigen
    
    ```bash
    sudo apt update	
    sudo apt install libeigen3-dev
    ```
    
- Ceres
    
    ```bash
    sudo apt update
    sudo apt install libeigen3-dev libgoogle-glog-dev libgflags-dev
    sudo apt install libceres-dev
    ```
    
- ompl
    
    ```bash
    sudo apt install libompl-dev
    ```
    

# Usage

If you want to test the algorithms without any additional installations, use the following commands:

```bash
git clone https://github.com/SEA-ME-COSS/Control.git
cd Control/algorithm
mkdir build && cd build
cmake ..
make
./control
```

# Reference

- [Hybrid A Star](https://github.com/bmegli/hybrid-a-star)
- [ROS2 foxy](https://docs.ros.org/en/foxy/index.html)