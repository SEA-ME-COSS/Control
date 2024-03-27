#pragma once

#include <planner/planner.h>

#include <queue>
#include <functional>
#include <cmath>


class A_Star : public Planner {
public:
    A_Star(Map& map);
    ~A_Star();

    virtual bool path_planning(std::vector<int> start_point, std::vector<int> end_point);
    virtual void save_planned_path(std::vector<int> start_point, std::vector<int> end_point);

private:
    int distance(std::vector<int> point1, std::vector<int> point2);
    int heuristic(std::vector<int> point, std::vector<int> end_point);
};

A_Star::A_Star(Map& map) : Planner(map) {}

A_Star::~A_Star() {}

bool A_Star::path_planning(std::vector<int> start_point, std::vector<int> end_point) {
    int count = 0;

    // Priority Queue for A* (cost & point)
    std::priority_queue<std::pair<int, std::vector<int>>, std::vector<std::pair<int, std::vector<int>>>, std::greater<std::pair<int, std::vector<int>>>> minimum_queue;

    // Validity Check
    if (map.is_blocked(start_point)) {return false;}
    if (map.is_blocked(end_point)) {return false;}

    // Initial Process
    minimum_queue.push(std::make_pair(this->heuristic(start_point, end_point), start_point));
    this->set_visit_and_prev(start_point, {0, 0});

    std::pair<int, std::vector<int>> old_cost_point;
    std::vector<int> old_point;
    int old_cost;
    std::pair<int, std::vector<int>> new_cost_point;
    std::vector<int> new_point;
    int new_cost;

    // For Test
    std::vector<std::vector<int>> new_points = {{-1,1},{0,1},{1,1},{-1,0},{1,0},{-1,-1},{0,-1},{1,-1}};
    while (!minimum_queue.empty()) {
        count += 1;

        old_cost = minimum_queue.top().first;
        old_point = minimum_queue.top().second;
        minimum_queue.pop();
        // std::cout<<old_point[0]<<"  "<<old_point[1]<<std::endl;
        for (int i = 0; i < 8; ++i) {
            new_point = {old_point[0]+new_points[i][0], old_point[1]+new_points[i][1]};
            if ((map.is_blocked(new_point)) || (this->is_visited(new_point))) {
                continue;
            }
            new_cost = this->distance(new_point, old_point) + this->heuristic(new_point, end_point) + old_cost;
            minimum_queue.push(std::make_pair(new_cost, new_point));
            this->set_visit_and_prev(old_point, new_point);

            if (new_point == end_point) {return true;}
        }
    }
    return false;
}

void A_Star::save_planned_path(std::vector<int> start_point, std::vector<int> end_point) {
    // std::cout<<"get_route"<<std::endl;
    std::vector<int> now_point = end_point;

    while (now_point != start_point) {
        // std::cout<<"x = "<<now_point[0]<<" y = "<<now_point[1]<<std::endl;
        now_point = this->get_prev_point(now_point);
        this->iteration_path.push_front(now_point);
    }
}

int A_Star::distance(std::vector<int> point1, std::vector<int> point2) {
    int distance = std::sqrt((point1[0] - point2[0])*(point1[0] - point2[0]) + (point1[1] - point2[1])*(point1[1] - point2[1]));
    // std::cout<<distance<<std::endl;
    return distance;
}


int A_Star::heuristic(std::vector<int> point, std::vector<int> end_point) {
    int heuristic = std::sqrt((end_point[0] - point[0])*(end_point[0] - point[0]) + (end_point[1] - point[1])*(end_point[1] - point[1]));
    // std::cout<<heuristic<<std::endl;
    return heuristic;
}