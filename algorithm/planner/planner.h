#pragma once

#include <utils/map.h>

#include <deque>
#include <array>
#include <utility>
#include <map>
#include <cstdint>

class Planner {
public:
    Planner(Map& map);
    ~Planner();

    void plan_with_waypoints(std::vector<std::vector<double>> waypoints);
    std::vector<std::vector<int>> get_waypoints_path();

protected:
    Map& map;

    // Deque(to Visit) and Map(Current & Previous) for Graph Search
    std::map<std::vector<int>, std::vector<int>> visit_map;

    void clear_storage();
    void set_visit_and_prev(std::vector<int> old_point, std::vector<int> new_point);
    bool is_visited(std::vector<int> point);
    std::vector<int> get_prev_point(std::vector<int> point);

    virtual bool path_planning(std::vector<int> start_point, std::vector<int> end_point) = 0;
    virtual void save_planned_path(std::vector<int> start_point, std::vector<int> end_point) = 0;

    std::deque<std::vector<int>> iteration_path;
    std::vector<std::vector<int>> waypoints_route;
};

Planner::Planner(Map& map) : map(map) {}

Planner::~Planner() {}

void Planner::clear_storage() {
    this->visit_map.clear();
    this->iteration_path.clear();
}

void Planner::plan_with_waypoints(std::vector<std::vector<double>> waypoints) {
    int waypoints_size = waypoints.size();

    std::cout<<"start_x : "<<waypoints[0][0]<<'\t'<<"start_y : "<<waypoints[0][1]<<std::endl;
    std::cout<<"end_x : "<<waypoints[waypoints_size-1][0]<<'\t'<<"end_y : "<<waypoints[waypoints_size-1][1]<<std::endl;

    for(int k = 0; k < (waypoints_size-1); ++k) {
        this->clear_storage();
            
        std::vector<int> start_point = {static_cast<int>(round(waypoints[k][0])), static_cast<int>(round(waypoints[k][1]))};
        std::vector<int> end_point = {static_cast<int>(round(waypoints[k+1][0])), static_cast<int>(round(waypoints[k+1][1]))};

        if (this->path_planning(start_point, end_point)) {
            this->save_planned_path(start_point, end_point);
            for (size_t i = 0; i < iteration_path.size(); ++i) {
                waypoints_route.push_back(iteration_path[i]);
            }
        }
        else {
            std::cout<<"No Way"<<std::endl;
            break;    
        }
    }
    // std::cout << "Plan Finish" << std::endl;
}

std::vector<std::vector<int>> Planner::get_waypoints_path() {
    return waypoints_route;
}

void Planner::set_visit_and_prev(std::vector<int> old_point, std::vector<int> new_point) {
    visit_map[new_point] = old_point;
}

bool Planner::is_visited(std::vector<int> point) {
    return visit_map.contains(point);
}

std::vector<int> Planner::get_prev_point(std::vector<int> point) {
    return visit_map[point];
}