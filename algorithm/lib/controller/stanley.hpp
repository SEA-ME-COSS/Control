#pragma once

#include "utils/car_struct.h"

#include <array>
#include <vector>
#include <cmath>
#include <iostream>
#include <limits>
#include <unistd.h>

class Stanley {

public:
    Stanley(std::vector<std::vector<double>> route, double resolution, double speed, std::vector<double> cpoint,
            double k, double ks);
    void stanley_control();
    std::vector<std::vector<double>> getTrajectory(); 
    std::vector<int> getTargetnode();

private:
    // Struct
    Car car;

    // Vehicle State    
    double x;
    double y;
    double yaw;
    double v;
    double acceleration;

    double front_x;
    double front_y;

    // PID 
    // double pid_integral;
    // double previous_error;
    double kp;
    // double ki;
    // double kd;

    // Vehicle Config
    double WB;

    // System Config
    double dt;

    // Stanley Parameters
    double k;
    double ks;

    // Path
    std::vector<std::vector<double>> route;
    int route_size;

    // Target Status
    double target_speed;
    int target_node;
    int previous_node;
    double delta;

    // etc
    double node_mindistance;
    double resolution;

    // Storage
    std::vector<std::vector<double>> trajectory;
    std::vector<int> target_nodes;

    bool isNearby(std::vector<double> cpoint, std::vector<double> target_point); 
    double pid_speed_control();
    void update_target_node();
    double stanley_steer_calc();
    double pi_2_pi(double angle);
    void update_state();
};