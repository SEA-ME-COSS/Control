#pragma once

#include <array>
#include <vector>
#include <cmath>
#include <iostream>
#include <unistd.h>

class PurePursuit {

public:
    PurePursuit(std::vector<std::vector<double>> route, double speed, std::vector<double> cpoint, double WB, double Kdd, double Ldc);

    void purepursuit_control();

    std::vector<std::vector<double>> getTrajectory();
    std::vector<int> getTargetnode();

private:
    // Vehicle State
    double x;
    double y;
    double yaw;
    double v;
    double acceleration;

    double rear_x;
    double rear_y;
    double WB;

    // PID 
    double pid_integral;
    double previous_error;
    double kp;
    // double ki;
    // double kd;

    double dt;

    // Target Node Searching
    double Ld;
    double Kdd;
    double Ldc;

    // purepursuit
    double delta;

    // Path
    std::vector<std::vector<double>> route;
    int route_size;

    // Target Status
    double target_speed;
    int target_node;

    std::vector<std::vector<double>> trajectory;
    std::vector<int> target_nodes;

    double pid_speed_control();
    double purepursuit_steer_calc();
    int update_target_node();
    void update_state();
    double calc_distance(double point_x, double point_y);
};
