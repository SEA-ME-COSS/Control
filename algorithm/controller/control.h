#pragma once

#include <array>
#include <cmath>
#include <iostream>
#include <unistd.h>

class PurePursuit {

public:
    PurePursuit(std::vector<std::vector<int>> route, double speed, std::vector<int> cpoint, double WB, double Kdd, double Ldc);

    void purepursuit_control();

    std::vector<std::vector<int>> getTrajectory();
    std::vector<int> getTargetnode();

private:
    // Vehicle State
    int x;
    int y;
    double yaw;
    double v;
    double acceleration;

    int rear_x;
    int rear_y;
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
    std::vector<std::vector<int>> route;
    int route_size;

    // Target Status
    double target_speed;
    int target_node;

    std::vector<std::vector<int>> trajectory;
    std::vector<int> target_nodes;

    double pid_speed_control();
    double purepursuit_steer_calc();
    int update_target_node();
    void update_state();
    double calc_distance(int point_x, int point_y);
};

PurePursuit::PurePursuit(std::vector<std::vector<int>> route, double speed, std::vector<int> cpoint, double WB, double Kdd, double Ldc) {
    // Vehicle Status Init
    this->x = cpoint[0];
    this->y = cpoint[1];
    this->yaw = cpoint[2];
    this->v = 0.0;
    this->acceleration = 0.0;

    this->WB = WB;
    this->rear_x = this->x - ((WB / 2) * cos(this->yaw));
    this->rear_y = this->y - ((WB / 2) * sin(this->yaw));

    // PID Init
    this->pid_integral = 0.0;
    this->previous_error = 0.0;
    this->kp = 1;
    // this->ki = 0.1;
    // this->kd = 0.01;
    
    this->dt = 0.1;

    // Target Node Searching Init
    this->Ldc = Ldc;
    this->Kdd = Kdd;
    this->Ld = this->Kdd * this->v + this->Ldc;

    // PurePursuit Init
    this->delta = 0.0;

    // Path Init
    this->route = route;
    this->route_size = route.size();

    // Target Status Init
    this->target_speed = speed;
    this->target_node = 0;
}

void PurePursuit::purepursuit_control() {
    while(this->target_node != this->route_size-1) {
        this->acceleration = this->pid_speed_control();
        this->delta = this->purepursuit_steer_calc();
        this->target_node = this->update_target_node();
        this->update_state();
    }
}

double PurePursuit::pid_speed_control() {
    // double error = target_speed - v;
    // pid_integral += error * dt;
    // double derivative = (error - previous_error)/dt;
    // double acceleration = kp * error + ki * pid_integral + kd * derivative;
    // previous_error = error;

    double acceleration = kp * (target_speed - v);

    return acceleration;
}

double PurePursuit::purepursuit_steer_calc() {
    int tx = route[this->target_node][0];
    int ty = route[this->target_node][1];

    double alpha = atan2(ty - this->rear_y, tx - this->rear_x) - this->yaw;
    double delta = atan2(2.0 * this->WB * sin(alpha)/this->Ld, 1.0);

    return delta;
}

int PurePursuit::update_target_node() {
    int cnode = this->target_node;
    if(cnode == this->route_size-1) { return cnode; }

    double distance_cnode = this->calc_distance(this->route[cnode][0], this->route[cnode][1]);
    double distance_nnode = 0.0;

    while (cnode < this->route.size()-1) {
        distance_nnode = this->calc_distance(this->route[cnode + 1][0], this->route[cnode + 1][1]);
        if (distance_cnode < distance_nnode) {
            break;
        }
        cnode += 1;
        distance_cnode = distance_nnode;
    }

    this->Ld = this->Kdd * this->v + this->Ldc;

    if(cnode < this->route_size-1) {
        while(this->Ld > this->calc_distance(this->route[cnode][0], this->route[cnode][1])) {
            cnode += 1;
        }
    }

    return cnode;
}

void PurePursuit::update_state() {
    x += v * cos(yaw) * dt;
    y += v * sin(yaw) * dt;
    yaw += v / WB * tan(delta) * dt;
    v += acceleration * dt;

    rear_x = x - ((WB / 2) * cos(yaw));
    rear_y = y - ((WB / 2) * sin(yaw));

    trajectory.push_back({x, y});
    target_nodes.push_back(target_node);
}

double PurePursuit::calc_distance(int point_x, int point_y) {
    double dx = this->rear_x - point_x;
    double dy = this->rear_y - point_y;
    
    return sqrt(dx * dx + dy * dy);
}

std::vector<std::vector<int>> PurePursuit::getTrajectory() {
    return trajectory;
}

std::vector<int> PurePursuit::getTargetnode() {
    return target_nodes;
}
