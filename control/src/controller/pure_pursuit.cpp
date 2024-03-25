#include "controller/pure_pursuit.hpp"

PurePursuit::PurePursuit(double WB, double Kdd, double Ldc) {
    // Vehicle Status Init
    // this->x = 0.0;
    // this->y = 0.0;
    // this->yaw = 0.0;
    // this->v = 0.0;
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

    // Target Status Init
    this->target_node = 0;
}

void PurePursuit::purepursuit_control(std::vector<Path> refPoses, double target_v, double x, double y, double yaw, double v) {
    this->target_speed = target_v;
    this->refPoses = refPoses;
    this->acceleration = this->pid_speed_control();
    this->delta = this->purepursuit_steer_calc();
    this->target_node = this->update_target_node();
    this->update_state(x, y, yaw, v);
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
    double tx = refPoses[this->target_node].x;
    double ty = refPoses[this->target_node].y;
    // double tyaw = refPoses[this->target_node].yaw;

    double alpha = atan2(ty - this->rear_y, tx - this->rear_x) - this->yaw;
    double delta = atan2(2.0 * this->WB * sin(alpha)/this->Ld, 1.0);

    return delta;
}

int PurePursuit::update_target_node() {
    int cnode = this->target_node;
    if(cnode == this->refPoses.size()-1) { return cnode; }

    double distance_cnode = this->calc_distance(this->refPoses[cnode].x, this->refPoses[cnode].y);
    double distance_nnode = 0.0;

    while (cnode < this->refPoses.size()-1) {
        distance_nnode = this->calc_distance(this->refPoses[cnode + 1].x, this->refPoses[cnode + 1].y);
        if (distance_cnode < distance_nnode) {
            break;
        }
        cnode += 1;
        distance_cnode = distance_nnode;
    }

    this->Ld = this->Kdd * this->v + this->Ldc;

    if(cnode < this->refPoses.size()-1) {
        while(this->Ld > this->calc_distance(this->refPoses[cnode].x, this->refPoses[cnode].y)) {
            cnode += 1;
        }
    }

    return cnode;
}

void PurePursuit::update_state(double x, double y, double yaw, double v) {
    this->x = x;
    this->y = y;
    this->yaw = yaw;
    this->v = v;

    rear_x = x - ((WB / 2) * cos(yaw));
    rear_y = y - ((WB / 2) * sin(yaw));
}

double PurePursuit::calc_distance(double point_x, double point_y) {
    double dx = this->rear_x - point_x;
    double dy = this->rear_y - point_y;
    
    return sqrt(dx * dx + dy * dy);
}

double PurePursuit::getThrottle() {
    return v;
}

double PurePursuit::getDelta() {
    return delta;
}