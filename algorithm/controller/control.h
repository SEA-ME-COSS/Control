#pragma once

#include <array>
#include <cmath>
#include <iostream>

class PurePursuit {

public:
    PurePursuit(std::vector<std::vector<int>> route, double speed, std::vector<int> cpoint, double WB, double k, double Lfc);

    void purepursuit_control();

    std::vector<std::vector<int>> getTrajectory();
    std::vector<int> getTargetnode();

private:
    // State
    int x;
    int y;
    double yaw;
    double v;
    int rear_x;
    int rear_y;
    double WB;

    // PID 
    double pid_integral;
    double dt;
    double previous_error;
    double kp;
    double ki;
    double kd;

    // node searching
    int old_nearest_node;
    double LF;
    double k;
    double Lfc;

    // purepursuit
    double delta;

    // Path
    std::vector<std::vector<int>> route;
    double target_speed;
    int target_node;

    std::vector<std::vector<int>> trajectory;
    std::vector<int> target_nodes;

    double pid_speed_control();
    int search_target_node();
    double calc_distance(int point_x, int point_y);
    void purepursuit_steer_calc();
    void update_state(double acceleration);
};

PurePursuit::PurePursuit(std::vector<std::vector<int>> route, double speed, std::vector<int> cpoint, double WB, double k, double Lfc) {
    // Path
    this->route = route;

    this->target_speed = speed;
    this->target_node = 0;
    this->WB = WB;
        
    this->x = cpoint[0];
    this->y = cpoint[1];
    this->yaw = cpoint[2];
    this->v = 0.0;
    this->rear_x = cpoint[0] - ((this->WB/2) * cos(cpoint[2]));
    this->rear_y = cpoint[1] - ((this->WB/2) * sin(cpoint[2]));

    // PID Init
    this->pid_integral = 0.0;
    this->dt = 0.1;
    this->previous_error = 0.0;
    this->kp = 1;
    this->ki = 0.1;
    this->kd = 0.01;

    this->old_nearest_node = 0;
    this->LF = 0.0;
    this->k = k;
    this->Lfc = Lfc;

}

void PurePursuit::purepursuit_control() {
    double accerlation = 0.0;
    while(this->target_node != this->route.size()) {
        accerlation = this->pid_speed_control();
        // std::cout << "A : " << x << std::endl;
        this->purepursuit_steer_calc();
        this->update_state(accerlation);
        // std::cout << "target node : " << this->x << std::endl;
    }
}

int PurePursuit::search_target_node() {
    int cnode = this->target_node;
    double distance_this_node = this->calc_distance(this->route[cnode][0], this->route[cnode][1]);
    double distance_next_node = 0.0;

    while (cnode + 1 < this->route.size()) {
        distance_next_node = this->calc_distance(this->route[cnode + 1][0], this->route[cnode + 1][1]);
        if (distance_this_node < distance_next_node) {
            break;
        }
        std::cout << "test" << std::endl;
        cnode += 1;
        distance_this_node = distance_next_node;
    }

    this->target_node = cnode; // 현재 가장 가까운 노드 업데이트

    double Lf = this->k * this->v + this->Lfc; // 전방 주시 거리 업데이트

    while (cnode + 1 < this->route.size() && Lf > this->calc_distance(this->route[cnode][0], this->route[cnode][1])) {
        std::cout << "test" << std::endl;
        cnode += 1;
    }

    std::cout << "old_nearest_node : " << this->target_node << std::endl;
    std::cout << "cnode : " << v << std::endl;

    return cnode;
}

void PurePursuit::purepursuit_steer_calc() {
    // int cnode = this->search_target_node();
    int tx = -1;
    int ty = -1;

    int new_target_node = this->search_target_node();
    if (this->target_node != new_target_node) {
        this->target_node = new_target_node; // target_node 업데이트
    }

    if(new_target_node < route.size()) {
        tx = route[new_target_node][0];
        ty = route[new_target_node][1];
    }
    else {
        tx = route[-1][0];
        ty = route[-1][0];
        new_target_node = route.size()-1;
    }

    double alpha = atan2(ty - this->rear_y, tx - this->rear_x) - this->yaw;
    this->delta = atan2(2.0 * this->WB * sin(alpha)/this->LF, 1.0);
}

void PurePursuit::update_state(double acceleration) {

    double newX = x + v * cos(yaw) * dt;
    double newY = y + v * sin(yaw) * dt;
    x = static_cast<int>(round(newX));
    y = static_cast<int>(round(newY));
    yaw += v/WB * tan(delta)*dt;
    v += acceleration*dt;
    rear_x = x - ((WB/2) * cos(yaw));
    rear_y = y - ((WB/2) * sin(yaw));

    trajectory.push_back({x, y});
    target_nodes.push_back(target_node);
}

double PurePursuit::calc_distance(int point_x, int point_y) {
    double dx = this->rear_x - point_x;
    double dy = this->rear_y - point_y;
    
    return sqrt(dx * dx + dy * dy);
}


double PurePursuit::pid_speed_control() {
    double error = target_speed - v;

    pid_integral += error * dt;

    double derivative = (error - previous_error)/dt;

    double accerlation = kp * error + ki * pid_integral + kd * derivative;

    previous_error = error;

    return accerlation;
}

std::vector<std::vector<int>> PurePursuit::getTrajectory() {
    return trajectory;
}

std::vector<int> PurePursuit::getTargetnode() {
    return target_nodes;
}
