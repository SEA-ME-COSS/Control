#pragma once

#include <string>
#include <vector>
#include <map>
#include <fstream>
#include <sstream>
#include <stdexcept>

std::vector<std::vector<int>> load_map_data(const std::string& file_path) {
    std::ifstream file(file_path);

    if (!file.is_open()) {
        throw std::runtime_error("Error opening file: " + file_path);
    }

    std::vector<std::vector<int>> map_data;
    std::string line;

    while (std::getline(file, line)) {
        std::istringstream iss(line);
        std::vector<int> row;
        int num;
        while (iss >> num) {
            row.push_back(num);
        }
        map_data.push_back(row);
    }

    file.close();

    return map_data;
}

std::map<int, std::vector<double>> load_nodes(const std::string& file_path) {
    std::ifstream file(file_path);

    if (!file.is_open()) {
        throw std::runtime_error("Error opening file: " + file_path);
    }

    std::map<int, std::vector<double>> nodes;
    std::string line;

    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::vector<double> nodeData;
        std::string value;
        while (std::getline(ss, value, ',')) {
            nodeData.push_back(std::stod(value)); 
        }
        nodes[static_cast<int>(nodeData[0])] = {nodeData[1], nodeData[2]};
    }

    file.close();

    return nodes;
}

std::vector<int> load_points(const std::string& file_path) {
    std::ifstream file(file_path);

    if (!file.is_open()) {
        throw std::runtime_error("Error opening file: " + file_path);
    }

    std::vector<int> points;
    std::string line;

    while (std::getline(file, line)) {
        points.push_back(std::stoi(line));
    }

    file.close();

    return points;
}

std::vector<std::vector<double>> load_waypoints(const std::string& nodes_file_path, const std::string& points_file_path) {
    std::map<int, std::vector<double>> nodes = load_nodes(nodes_file_path);
    std::vector<int> points = load_points(points_file_path);

    // make waypoints to actual coordinate
    std::vector<std::vector<double>> waypoints;
    for (int& point_id : points) {
        double node_x = nodes[point_id][0];
        double node_y = nodes[point_id][1];
        waypoints.push_back({node_x, node_y});
    }

    return waypoints;
}