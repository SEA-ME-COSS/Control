#pragma once

#include "msg_structs.h"

#include <array>
#include <cmath>
#include <iostream>
#include <vector>

class PurePursuit {

public:
    PurePursuit(double WB, double Kdd, double Ldc);

    void purepursuit_control(std::vector<Path> refPoses, double target_v, double x, double y, double yaw, double v);

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
    std::vector<Path> refPoses;
    int route_size;

    // Target Status
    double target_speed;
    int target_node;

    double pid_speed_control();
    double purepursuit_steer_calc();
    int update_target_node();
    void update_state(double x, double y, double yaw, double v);
    double calc_distance(double point_x, double point_y);
};
