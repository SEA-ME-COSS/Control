#include "planner/hybrid_astar_planner.hpp"

Planner::Planner(Map* map, double vehicle_width, double vehicle_length,
                float change_penalty, float non_straight_penalty, float reverse_penalty, float minimum_turning_radius, int theta_resolution,
                int max_iterations, float tolerance, int it_on_approach) {
    this->map = map;
    
    this->footprint.push_back( {-vehicle_width/2, -vehicle_length/2} );
    this->footprint.push_back( {vehicle_width/2, -vehicle_length/2} );
    this->footprint.push_back( {vehicle_width/2, vehicle_length/2});
    this->footprint.push_back( {-vehicle_width/2, vehicle_length/2} );

	this->info.change_penalty = change_penalty;
	this->info.non_straight_penalty = non_straight_penalty;
	this->info.reverse_penalty = reverse_penalty;
	this->info.minimum_turning_radius = minimum_turning_radius;
	this->theta_resolution = theta_resolution;

	this->max_iterations = max_iterations;
	this->tolerance = tolerance;
    this->it_on_approach = it_on_approach;

    this->target_node = 0;

    this->smoother = new nav2_smac_planner::Smoother<Map>();
    this->optimizer_params.debug = false;
    this->smoother->initialize(this->optimizer_params);
    this->smoother_params.max_curvature = 1.0f / minimum_turning_radius;
	this->smoother_params.curvature_weight = 30.0;
	this->smoother_params.distance_weight = 0.0;
	this->smoother_params.smooth_weight = 100.0;
	this->smoother_params.costmap_weight = 0.025;
	this->smoother_params.max_time = 0.1;
}

Planner::~Planner() {
}

void Planner::plan_route(std::vector<std::array<int, 3>> waypoints) {
    std::array<int, 3> startPose;
    std::array<int, 3> goalPose;
    int num_it = 0;
    std::vector<int> tempState = {0,0,0};

    for(int k = this->target_node; (k + 1) < static_cast<int>(waypoints.size()); ++k) {
        startPose = waypoints[k];
        goalPose = waypoints[k+1];

        // TODO: Exception Handling (About Block)
        if (!map->isValid(startPose[0], startPose[1])) {std::cout<<"Start Error"<<std::endl;}
        if (!map->isValid(goalPose[0], goalPose[1])) {std::cout<<"Goal Error"<<std::endl;}

        nav2_smac_planner::AStarAlgorithm<Map, nav2_smac_planner::GridCollisionChecker<Map, Point>> a_star(nav2_smac_planner::MotionModel::REEDS_SHEPP, info);

        a_star.initialize(false, max_iterations, it_on_approach);
        a_star.setFootprint(footprint, true);
        a_star.createGraph(map->getSizeInCellsX(), map->getSizeInCellsY(), 360 / theta_resolution, map);

        a_star.setStart(startPose[0], startPose[1], startPose[2] / theta_resolution);
        a_star.setGoal(goalPose[0], goalPose[1], goalPose[2] / theta_resolution);

        nav2_smac_planner::NodeSE2::CoordinateVector plan;
        bool found = a_star.createPath(plan, num_it, tolerance);
        if (!found) {std::cout<<"NO WAY"<<std::endl;}

        std::vector<Eigen::Vector2d> smooth_path;
        for (const auto& node : plan) {
            smooth_path.push_back(Eigen::Vector2d(node.x, node.y));
        }

        if (!smoother->smooth(smooth_path, map, smoother_params)) {
            std::cout << "Smoothing Fail" << std::endl;
        }

        for (size_t i = smooth_path.size()-1; 0<i; --i) {
            waypoints_route.push_back({smooth_path[i].x(), smooth_path[i].y(), plan[i].theta * M_PI / 180 * theta_resolution});
        }

        // for (size_t i = plan.size()-1; 0 < i; --i) {
        //     waypoints_route.push_back({plan[i].x, plan[i].y, plan[i].theta * M_PI / 180 * theta_resolution});
        // }
    }
}

std::vector<std::vector<double>> Planner::get_route() {
    // std::cout << "Waypoints route:" << std::endl;
    // for (const auto& waypoint : waypoints_route) {
    //     std::cout << "x: " << waypoint[0] << ", y: " << waypoint[1] << ", theta: " << waypoint[2] << std::endl;
    // }
    return waypoints_route;
}