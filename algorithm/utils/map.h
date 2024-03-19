#pragma once

#include <utils/loader.h>

#include <vector>
#include <cmath>
#include <iostream>

class Map {
public:
    Map(const std::string& map_path);
    ~Map();

    std::vector<std::vector<int>> get_grid_map();
    std::vector<std::vector<bool>> get_binary_map();
    std::vector<int> get_map_info();

    bool is_blocked(std::vector<int> point);

private:
    int rows;
    int cols;

    bool is_outside(std::vector<int> point);
    void set_grid_map(std::vector<std::vector<int>> *map_data);
    void set_binary_map(std::vector<std::vector<int>> *map_data);

    std::vector<std::vector<int>> grid_map;
    std::vector<std::vector<bool>> booleanMapData;
};

Map::Map(const std::string& map_path) {
    std::vector<std::vector<int>> map_data = load_map_data(map_path);
    this->rows = map_data[0].size();
    this->cols = map_data.size();

    this->set_grid_map(&map_data);
    this->set_binary_map(&map_data);

    std::cout << "Map Init Success" << std::endl;
}

Map::~Map() {}

void Map::set_grid_map(std::vector<std::vector<int>> *map_data) {
    this->grid_map.resize(rows, std::vector<int>(cols, 0));

    for (int x = 0; x < rows; ++x) {
        for (int y = 0; y < cols; ++y) {
            grid_map[x][y] = (*map_data)[y][x];
        }
    }
    // std::cout << "Grid Map Size: " << grid_map.size() << " x " << grid_map[0].size() << std::endl;  
}

void Map::set_binary_map(std::vector<std::vector<int>> *map_data) {
    this->booleanMapData.resize(rows, std::vector<bool>(cols, false));

    // Make Binary Map
    for(int y=0; y<cols; ++y) {
        for(int x=0; x<rows; ++x) {
            uint8_t map_value = (*map_data)[y][x];
            switch (map_value) {
                case 0: // Driveway
                case 2: // Parking Spot
                case 3: // Dotted Line
                case 4: // Stop Line
                case 5: // Crosswalk
                    // CAN
                    booleanMapData[x][y] = false;
                    break;
                case 1: // Solid Line
                case 6: // Roundabout
                    // CANNOT
                    booleanMapData[x][y] = true;
                    break;
                default:
                    std::cerr << "Invalid binary_occupancy_map value: " << static_cast<int>(map_value) << " at (" << x << ", " << y << ")\n";
                    exit(EXIT_FAILURE);
                    break;
            }
        }
    }

    // Make the lanes thicker
    int extra = 16;
    for(int y=0; y<cols; ++y) {
        for(int x=0; x<rows; ++x) {
            uint8_t map_value = (*map_data)[y][x];
            switch (map_value) {
                case 0: // Driveway
                case 2: // Parking Spot
                case 3: // Dotted Line
                case 4: // Stop Line
                case 5: // Crosswalk
                    // CAN
                    break;
                case 1: // Solid Line
                case 6: // Roundabout
                    // CANNOT
                    for (int dx = -extra; dx <= extra; ++dx) {
                        for (int dy = -extra; dy <= extra; ++dy) {
                            int nx = x + dx;
                            int ny = y + dy;
                            if (dx == 0 && dy == 0) {continue;}
                            if (nx < 0 || nx >= rows || ny < 0 || ny >= cols) {continue;} 
                            if (booleanMapData[nx][ny]) {continue;}
                            if (std::sqrt(std::pow(nx-x,2) + std::pow(ny-y,2))>extra) {continue;}
                            booleanMapData[nx][ny] = true;
                            // grid_map[nx][ny] = 10;
                        }
                    }
                    break;
                default:
                    std::cerr << "Invalid binary_occupancy_map value: " << static_cast<int>(map_value) << " at (" << x << ", " << y << ")\n";
                    exit(EXIT_FAILURE);
                    break;
            }
        }
    }
    // std::cout << "Binary Map Size: " << grid_map.size() << " x " << grid_map[0].size() << std::endl;  
}

std::vector<std::vector<int>> Map::get_grid_map() {
    return grid_map;
}

std::vector<std::vector<bool>> Map::get_binary_map() {
    return booleanMapData;
}

std::vector<int> Map::get_map_info() {
    std::vector<int> map_info = {rows, cols};
    return map_info;
}

bool Map::is_blocked(std::vector<int> point) {
    // Is outside
    if (point[0] < 0 || point[0] >= rows || point[1] < 0 || point[1] >= cols) {
        return true;
    }

    // If blocked true
    return booleanMapData[point[0]][point[1]];
}
