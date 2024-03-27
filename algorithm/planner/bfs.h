#pragma once

#include <planner/planner.h>

class BFS : public Planner {
public:
    BFS(Map& map);
    ~BFS();

    virtual bool path_planning(std::vector<int> start_point, std::vector<int> end_point);
    virtual void save_planned_path(std::vector<int> start_point, std::vector<int> end_point);
};

BFS::BFS(Map& map) : Planner(map) {
}

BFS::~BFS() {}

bool BFS::path_planning(std::vector<int> start_point, std::vector<int> end_point) {
    int count = 0;

    std::deque<std::vector<int>> visit_deque;

    // Validity Check
    if (map.is_blocked(start_point)) {return false;}
    if (map.is_blocked(end_point)) {return false;}

    // Initial Process
    visit_deque.push_back(start_point);
    this->set_visit_and_prev(start_point, {0, 0});

    std::vector<int> old_point;
    std::vector<int> new_point;

    while (!visit_deque.empty()) {
        count += 1;

        old_point = visit_deque.front();
        visit_deque.pop_front();
        
        std::vector<std::vector<int>> new_points = {{old_point[0]+1,old_point[1]}, {old_point[0], old_point[1]+1}, {old_point[0]-1, old_point[1]}, {old_point[0], old_point[1]-1}};

        for (int i = 0; i < 4; ++i) {
            new_point = new_points[i];
            if ((map.is_blocked(new_point)) || (this->is_visited(new_point))) {
                continue;
            }
            visit_deque.push_back({new_point});
            this->set_visit_and_prev(old_point, new_point);

            if (new_point == end_point) {return true;}
        }
        // if (count++<max_iter) {
        //     std::cout<<"Maximum Iteration : "<<count<<std::endl;
        //     return false;
        // }
    }
    return false;
}

void BFS::save_planned_path(std::vector<int> start_point, std::vector<int> end_point) {
    // std::cout<<"get_route"<<std::endl;
    std::vector<int> now_point = end_point;

    while (now_point != start_point) {
        now_point = this->get_prev_point(now_point);
        this->iteration_path.push_front(now_point);
    }
}