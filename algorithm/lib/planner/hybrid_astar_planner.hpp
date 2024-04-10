#pragma once

#include "map/hybrid_astar_map.hpp"
#include "hybrid-a-star/a_star.hpp"
#include "hybrid-a-star/smoother.hpp"
#include "utils/hybrid_astar_struct.h"

#include <iostream>
#include <array>
#include <chrono>

class Planner {
public:
    Planner(Map* map, double vehicle_width, double vehicle_length,
            float change_penalty, float non_straight_penalty, float reverse_penalty, float minimum_turning_radius, int theta_resolution,
            int max_iterations, float tolerance, int it_on_approach);
    ~Planner();

    void plan_route(std::vector<std::array<int, 3>> waypoints);
    std::vector<std::vector<double>> get_route();

private:
    Map* map;
    fcc::FootprintCollisionChecker<Map, Point>::Footprint footprint;
    nav2_smac_planner::SearchInfo info;
    unsigned int theta_resolution;

	int max_iterations;
	float tolerance;
	int it_on_approach;

    std::vector<std::vector<double>> waypoints_route;
    int target_node;

    nav2_smac_planner::Smoother<Map>* smoother;
    nav2_smac_planner::OptimizerParams optimizer_params;
    nav2_smac_planner::SmootherParams smoother_params;
};