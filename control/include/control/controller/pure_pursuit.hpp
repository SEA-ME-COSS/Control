#pragma once

#include <array>
#include <cmath>
#include <iostream>
#include <unistd.h>

class PurePursuit {

public:
    PurePursuit(std::vector<std::vector<int>> route, double speed, std::vector<int> cpoint, double WB, double Kdd, double Ldc);

    void purepursuit_control();

    double getThrottle();
    double getDelta();

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

    double pid_speed_control();
    double purepursuit_steer_calc();
    int update_target_node();
    void update_state(double x, double y, double yaw, double v);
    double calc_distance(int point_x, int point_y);
};
