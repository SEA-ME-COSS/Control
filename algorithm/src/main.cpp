#include "planner/hybrid_astar_planner.hpp"
#include "map/hybrid_astar_map.hpp"
#include "hybrid-a-star/a_star.hpp"

#include "controller/pure_pursuit.hpp"
#include "controller/stanley.hpp"

#include "utils/matplotlibcpp.h"
#include "utils/loader.hpp"

#include <iostream>
#include <cmath>
#include <vector>
#include <fstream>
#include <sstream>

namespace plt = matplotlibcpp;

int main() {
    // Load Graph and Waypoints
    std::string nodes_file_path = "../globalmap/parsinginfo.txt";
    std::string points_file_path = "../globalmap/waypoints.txt";
    std::string points_set_file_path = "../globalmap/waypoints-set.txt";
    std::vector<std::array<int, 3>> waypoints = load_waypoints(nodes_file_path, points_file_path);
    std::vector<std::array<int, 3>> waypoints_set = load_waypoints(nodes_file_path, points_set_file_path);

    // Load and Configure Map
    std::string mapdata_file_path = "../globalmap/flipped-track.txt";
    double resolution = 0.77;
    Map *map = new Map(mapdata_file_path, resolution, waypoints_set);

    double vehicle_width = 15.0;
    double vehicle_length = 30.0;
    float change_penalty = 3.0;
	float non_straight_penalty = 2.0;
	float reverse_penalty = 2.0;
	float minimum_turning_radius = 60.0;
	int theta_resolution = 5;

	int max_iterations = 10000;
	float tolerance = 10.0;
	int it_on_approach = 10;
    
    Planner planner = Planner(map, vehicle_width, vehicle_length,
                            change_penalty, non_straight_penalty, reverse_penalty, minimum_turning_radius, theta_resolution,
                            max_iterations, tolerance, it_on_approach);

    planner.plan_route(waypoints);
    std::vector<std::vector<double>> route = planner.get_route();

    std::vector<double> cpoint = {130.0, 64.0, 0};
    double speed = 5.0/resolution;
    double WB = 16/resolution;
    double Kdd = 5/resolution;
    double Ldc = 8/resolution;

    // auto purepursuit = PurePursuit(route, speed, cpoint, WB, Kdd, Ldc);
    // purepursuit.purepursuit_control();

    // std::vector<std::vector<double>> control_path = purepursuit.getTrajectory();
    // std::vector<int> target_nodes = purepursuit.getTargetnode();

    double k = 0.2;
    double ks = 1.0;

    auto stanley = Stanley(route, resolution, speed, cpoint, k, ks);
    stanley.stanley_control();

    std::vector<std::vector<double>> control_path = stanley.getTrajectory();
    std::vector<int> target_nodes = stanley.getTargetnode();

    // // Only for drawing
    std::cout << "Draw Map" << std::endl;
    std::map<int, float> structure_color_map = {
        {0, 1.0},
        {255, 0.0},
        {254, 0.1},
        {253, 0.7},
        {252, 0.8},
    };

    // // Draw map info
    std::vector<std::vector<unsigned int>> draw_grid_map = map->get_cost_map();
    std::vector<double> draw_map_info = map->get_map_info();

    int size_x = static_cast<int>(draw_map_info[0]);
    int size_y = static_cast<int>(draw_map_info[1]);

    std::vector<float> image_data(size_x * size_y);

    for(int x = 0; x < size_x; ++x) {
        for(int y = 0; y < size_y; ++y) {
            int structure = draw_grid_map[y][x];
            int index = y * size_x + x;
            image_data[index] = structure_color_map[structure];
        }
    }

    plt::imshow(&image_data[0], size_y, size_x, 1);

    // Draw global path : reverse mode
    std::vector<int> path_x;
    std::vector<int> path_y;

    for (const auto& waypoint : route) {
        path_x.push_back(waypoint[0]);
        path_y.push_back(waypoint[1]);
    }

    for(size_t i = 0; i < control_path.size(); i += 5) {

        plt::clf();
        plt::imshow(&image_data[0], size_y, size_x, 1);

        // Show path
        plt::plot(path_x, path_y, "r-");

        // Show Control
        if(i < control_path.size()) {
            std::vector<int> path_x = {static_cast<int>(control_path[i][0])};
            std::vector<int> path_y = {static_cast<int>(control_path[i][1])};
            plt::plot(path_x, path_y, "bo");
        }

        if(i < target_nodes.size()) {
            int target_node_index = target_nodes[i];
            if(target_node_index < route.size()) {
                std::vector<int> target_x = {static_cast<int>(route[target_node_index][0])};
                std::vector<int> target_y = {static_cast<int>(route[target_node_index][1])};
                plt::plot(target_x, target_y, "go");
            }
        }

        plt::pause(0.01);
    }

    plt::show();

    return 0;
}