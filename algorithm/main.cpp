#include <utils/map.h>
#include <utils/loader.h>
#include <visual/matplotlibcpp.h>

#include <planner/bfs.h>
#include <planner/a_star.h>

#include <controller/control.h>

#include <iostream>
#include <cmath>
#include <fstream>
#include <sstream>

namespace plt = matplotlibcpp;

int main() {
    // Load Waypoints Info
    std::string nodes_file_path = "../globalmap/parsinginfo.txt";
    std::string waypoints_file_path = "../globalmap/waypoints.txt";
    std::vector<std::vector<double>> waypoints = load_waypoints(nodes_file_path, waypoints_file_path);

    // Make Map Instruction
    std::string map_file_path = "../globalmap/flipped-track.txt";
    Map map = Map(map_file_path);

    // Path Planner (Using same map)
    // auto planner = BFS(map);
    auto planner = A_Star(map);
    planner.plan_with_waypoints(waypoints);
    std::vector<std::vector<int>> route = planner.get_waypoints_path();

    std::vector<int> cpoint = {130, 64, 0};

    auto purepursuit = PurePursuit(route, 20.0, cpoint, 2.7, 0.5, 4.0);
    purepursuit.purepursuit_control();

    std::vector<std::vector<int>> control_path = purepursuit.getTrajectory();
    std::vector<int> target_nodes = purepursuit.getTargetnode();

    // Only for drawing
    std::cout << "Draw Map" << std::endl;
    std::map<int, float> structure_color_map = {
        {0, 1.0}, // Driveway
        {1, 0.5}, // Solid Line
        {2, 0.3}, // Parking Spot
        {3, 0.4}, // Dotted Line
        {4, 0.5}, // Stop Line
        {5, 0.6}, // Crosswalk
        {6, 0.7}, // Roundabout
        {10, 0.9}, // Extra
    };

    // Draw map info
    std::vector<std::vector<int>> draw_grid_map = map.get_grid_map();
    std::vector<int> draw_map_info = map.get_map_info();

    int map_width = draw_map_info[0];
    int map_height = draw_map_info[1];

    std::vector<float> image_data(map_width * map_height);

    for(int x=0; x<map_width; ++x) {
        for(int y=0; y<map_height; ++y) {
            int structure = draw_grid_map[x][y];
            int index = y * map_width + x;
            image_data[index] = structure_color_map[structure];
        }
    }

    plt::imshow(&image_data[0], map_height, map_width, 1);

    // Draw global path : reverse mode
    std::vector<int> path_x;
    std::vector<int> path_y;

    for (const auto& waypoint : route) {
        path_x.push_back(waypoint[0]);
        path_y.push_back(waypoint[1]);
    }

    for(size_t i = 0; i < control_path.size(); i += 5) {

        plt::clf();
        plt::imshow(&image_data[0], map_height, map_width, 1);

        // Show path
        plt::plot(path_x, path_y, "r-");

        // Show Control
        if(i < control_path.size()) {
            std::vector<int> path_x = {control_path[i][0]};
            std::vector<int> path_y = {control_path[i][1]};
            plt::plot(path_x, path_y, "bo");
        }

        if(i < target_nodes.size()) {
            int target_node_index = target_nodes[i];
            if(target_node_index < route.size()) {
                std::vector<int> target_x = {route[target_node_index][0]};
                std::vector<int> target_y = {route[target_node_index][1]};
                plt::plot(target_x, target_y, "go"); // 목표 노드는 초록색으로
            }
        }

        plt::pause(0.01);
    }

    plt::show();

    return 0;
}