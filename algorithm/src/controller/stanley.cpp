#include "controller/stanley.hpp"

Stanley::Stanley(std::vector<std::vector<double>> route, double resolution, double speed, std::vector<double> cpoint,
                double k, double ks) {
    // Vehicle Config
    this->WB = this->car.WB/resolution;

    // Vehicle Status
    this->x = cpoint[0];
    this->y = cpoint[1];
    this->yaw = cpoint[2];
    this->v = 0.0;
    this->acceleration = 0.0;

    this->front_x = this->x + this->WB*cos(this->yaw);
    this->front_y = this->y + this->WB*sin(this->yaw);

    // PID Config
    // this->pid_integral = 0.0;
    // this->previous_error = 0.0;
    this->kp = this->car.kp;
    // this->ki = this->car.ki;
    // this->kd = this->car.kd;

    // System Config
    this->dt = 0.1;

    // Stanley Parameters
    this->k = k;
    this->ks = ks;

    // Path
    this->route = route;
    this->route_size = route.size();

    // Target
    this->target_speed = speed;
    this->target_node = 0;
    this->previous_node = 0;
    this->delta = 0.0;

    // etc
    this->node_mindistance = 5.0/resolution;
    this->resolution = resolution;
}

void Stanley::stanley_control() {
    while(!isNearby({this->x, this->y}, this->route.back())) {
        this->acceleration = this->pid_speed_control();
        this->update_target_node();
        this->delta = this->stanley_steer_calc();
        this->update_state();
    }
}

bool Stanley::isNearby(std::vector<double> cpoint, std::vector<double> target_point) {
    double distance = std::pow(cpoint[0]-target_point[0], 2) + std::pow(cpoint[1]-target_point[1], 2);

    return distance < this->node_mindistance;
}

double Stanley::pid_speed_control() {
    // double error = target_speed - v;
    // pid_integral += error * dt;
    // double derivative = (error - previous_error)/dt;
    // double acceleration = kp * error + ki * pid_integral + kd * derivative;
    // previous_error = error;

    double acceleration = kp * (target_speed - v);

    return acceleration;
}

void Stanley::update_target_node() {
    int closest_node = -1;
    int second_closest_node = -1;
    double min_distance = std::numeric_limits<double>::max();
    double second_min_distance = std::numeric_limits<double>::max();

    for (size_t i = 0; i < this->route_size; ++i) {
        double dx = this->front_x - this->route[i][0];
        double dy = this->front_y - this->route[i][1];
        double distance = std::pow(dx, 2) + std::pow(dy, 2);

        if (distance < min_distance) {
            second_min_distance = min_distance;
            second_closest_node = closest_node;
            min_distance = distance;
            closest_node = i;
        } 
        else if (distance < second_min_distance) {
            second_min_distance = distance;
            second_closest_node = i;
        }
    }

    this->target_node = std::max(closest_node, second_closest_node);
}


double Stanley::stanley_steer_calc() {
    double dx = this->front_x - route[this->target_node][0];
    double dy = this->front_y - route[this->target_node][1];

    double front_axle_vec_rot_90_x = cos(yaw - M_PI/2.0);
    double front_axle_vec_rot_90_y = sin(yaw - M_PI/2.0);

    double e = dx * front_axle_vec_rot_90_x + dy * front_axle_vec_rot_90_y;

    double theta_e = pi_2_pi(route[target_node][2] - yaw);

    double delta = theta_e + atan2(k*e, v+ks);

    return delta;
}

double Stanley::pi_2_pi(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

void Stanley::update_state() {
    x += v * cos(yaw) * dt;
    y += v * sin(yaw) * dt;
    yaw += v / WB * tan(delta) * dt;
    v += acceleration * dt;

    front_x = x + WB*cos(yaw);
    front_y = y + WB*sin(yaw);

    trajectory.push_back({x, y});
    target_nodes.push_back(target_node);
}

std::vector<std::vector<double>> Stanley::getTrajectory() {
    // for (const auto& point : trajectory) {
    //     std::cout << "x: " << point[0] << ", y: " << point[1] << std::endl;
    // }
    return trajectory;
}

std::vector<int> Stanley::getTargetnode() {
    // for (const int& node : target_nodes) {
    //     std::cout << "Target Node : " << node << std::endl;;
    // }
    return target_nodes;
}
