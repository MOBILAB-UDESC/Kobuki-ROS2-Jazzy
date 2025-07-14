#include "kobuki_planner/HybridAstar.hpp"

HybridAStarPlanner::HybridAStarPlanner(std::vector<std::vector<unsigned char>> map,
                                        double map_resolution,
                                        float robot_radius,
                                        std::vector<double> origin,
                                        std::vector<double> start_pose,
                                        std::vector<double> goal_pose)
{
    this->origin         = origin;
    this->map            = map;
    this->height         = map.size();
    this->width          = map[0].size();
    this->map_resolution = map_resolution;
    this->robot_radius   = robot_radius;
    this->start_pose     = start_pose;
    this->goal_pose      = goal_pose;

    this->precompute_heuristic();
};

bool HybridAStarPlanner::is_in_map(double x, double y) const {
    std::vector<int> map_coords = MtoP({x, y}, this->origin, this->height, this->map_resolution);
    return map_coords[0] >= 0 && map_coords[0] < this->width &&
           map_coords[1] >= 0 && map_coords[1] < this->height;
};

bool HybridAStarPlanner::is_collision(double x, double y) const {

    std::vector<int> map_coords = MtoP({x, y}, this->origin, this->height, this->map_resolution);

    int pixel_safety_x = static_cast<int>(this->robot_radius / this->map_resolution);
    int pixel_safety_y = pixel_safety_x;

    for (int dx = -pixel_safety_x; dx <= pixel_safety_x; ++dx) {
        for (int dy = -pixel_safety_y; dy <= pixel_safety_y; ++dy) {
            int px = map_coords[0] + dx;
            int py = map_coords[1] + dy;

            // if (std::sqrt(dx*dx + dy*dy) <= pixel_safety_x) {
            if (px >= 0 && px < this->width && py >= 0 && py < this->height) {
                if (this->map[py][px] == 0) {
                    return true;
                }
            } else {
                return true;
            }
            // }
        }
    }

    return false;
};

bool HybridAStarPlanner::is_collision(int x, int y) const {

    int pixel_safety_x = static_cast<int>(this->robot_radius / this->map_resolution);
    int pixel_safety_y = pixel_safety_x;

    for (int dx = -pixel_safety_x; dx <= pixel_safety_x; ++dx) {
        for (int dy = -pixel_safety_y; dy <= pixel_safety_y; ++dy) {
            int px = x + dx;
            int py = y + dy;

            // if (std::sqrt(dx*dx + dy*dy) <= pixel_safety_x) {
            if (px >= 0 && px < this->width && py >= 0 && py < this->height) {
                if (this->map[py][px] == 0) {
                    return true;
                }
            } else {
                return true;
            }
            // }
        }
    }

    // std::cout << "salió" << std::endl;
    return false;
}

std::shared_ptr<HybridAStarPlanner::Node> HybridAStarPlanner::get_neighbour(std::shared_ptr<HybridAStarPlanner::Node> current_node, float v, float w)
{

    // Kobuki's dynamics
    double next_x, next_y, next_theta;

    next_x = current_node->x + v * this->DT * std::cos(current_node->theta);
    next_y = current_node->y + v * this->DT * std::sin(current_node->theta);

    if(this->is_collision(next_x, next_y)) {
        return nullptr;
    }

    next_theta = normalize_angle(current_node->theta + w * this->DT);

    std::shared_ptr<Node> next_node = std::make_shared<Node>();

    // Add to Node
    next_node->x     = next_x;
    next_node->y     = next_y;
    next_node->theta = next_theta;
    next_node->parent = current_node;
    next_node->parent_input = {v, w};
    next_node->h_cost = heuristic(next_x, next_y, next_theta)*this->DT;

    double incremental_cost = 0.0;

    double distance_traveled = std::sqrt(std::pow(next_x - current_node->x, 2) + std::pow(next_y - current_node->y, 2));
    incremental_cost += distance_traveled * this->forward_cost;

    if (v < 0) {
        incremental_cost *= this->reverse_cost;
    }

    double angle_change = std::abs(normalize_angle(next_theta - current_node->theta));
    incremental_cost += angle_change * this->turn_cost;

    std::vector<int> next_pixel = MtoP({next_x, next_y}, this->origin, this->height, this->map_resolution);
    int next_px = next_pixel[0];
    int next_py = next_pixel[1];

    if (next_px >= 0 && next_px < this->width && next_py >= 0 && next_py < this->height) {
        unsigned char map_cell_value = this->map[next_py][next_px];
        double cell_cost = 0.0;
        if (map_cell_value > 0) { // Si no es libre
            cell_cost = static_cast<double>(map_cell_value) / 255.0;
        }
        
        incremental_cost += distance_traveled * cell_cost * this->maps_cost;
    } else {
        incremental_cost = std::numeric_limits<double>::max(); 
    }

    next_node->g_cost = current_node->g_cost + incremental_cost;
   
    return next_node;
}

double HybridAStarPlanner::heuristic(double x, double y, double theta) const
{
    // Manhattan
    // return 0.8*(std::abs(x - this->goal_pose[0]) + std::abs(y - this->goal_pose[1])) + 0.2*(std::abs(theta - this->goal_pose[2]));
    
    // Euclidean
    // return 0.8*(std::sqrt(std::pow(x - this->goal_pose[0], 2) + std::pow(y - this->goal_pose[1], 2))) + 0.2*(std::abs(theta - this->goal_pose[2]));

    // Heuristic based on the precomputed heuristic map
    std::vector<int> map_coords = MtoP({x, y}, this->origin, this->height, this->map_resolution);
    int map_x = map_coords[0];
    int map_y = map_coords[1];

    return heuristic_map[map_x][map_y]; 

}

void HybridAStarPlanner::precompute_heuristic()
{
    // Backwards Djikstra's algorithm to precompute the heuristic map of every point in the map
    this->heuristic_map.clear();

    // W, NW, N, NE, E, SE, S, NS
    const int dx[8]      = {-1,    -1,  0,     1,  1,     1,  0,    -1};
    const int dy[8]      = { 0,     1,  1,     1,  0,    -1, -1,    -1};
    const float costs[8] = {1., 1.414, 1., 1.414, 1., 1.414, 1., 1.414};

    // Initialize the heuristic map with maximum values
    heuristic_map.assign(this->height, std::vector<double>(this->width, std::numeric_limits<double>::max()));

    std::vector<int> goal_pixel = MtoP({goal_pose[0], goal_pose[1]}, this->origin, this->height, this->map_resolution);
    int goal_px = goal_pixel[0];
    int goal_py = goal_pixel[1];

    // Create the Node structure
    struct HeuristicNode {
        int x_pixel, y_pixel;
        double cost;
        bool operator>(const HeuristicNode& other) const { return cost > other.cost; }
    };

    // Priority queue for the heuristic map
    std::priority_queue<HeuristicNode, std::vector<HeuristicNode>, std::greater<HeuristicNode>> pq;

    heuristic_map[goal_py][goal_px] = 0.0;
    pq.push({goal_px, goal_py, 0.0});

    while (!pq.empty()) {
        HeuristicNode current = pq.top();
        pq.pop();

        int px = current.x_pixel;
        int py = current.y_pixel;
        double cost = current.cost;

        if (cost > this->heuristic_map[py][px]) {
            continue;
        }

        for (int i = 0; i < 8; ++i) {
            int next_px = px + dx[i];
            int next_py = py + dy[i];

            if(this->is_collision(next_px, next_py)) {
                continue;
            }

            if (next_px >= 0 && next_px < this->width && next_py >= 0 && next_py < this->height) {

                double new_cost = cost + costs[i];

                if (map[next_py][next_px] == 0) {
                    continue;
                }

                if (new_cost < this->heuristic_map[next_py][next_px]) {
                    this->heuristic_map[next_py][next_px] = new_cost;
                    pq.push({next_px, next_py, new_cost});
                }
            }
        }
    }
}

std::shared_ptr<HybridAStarPlanner::Node> HybridAStarPlanner::findPath()
{
    
    if (this->is_collision(this->goal_pose[0], this->goal_pose[1])) {
        std::cout << "collision" << std::endl;
        return nullptr;
    }

    if (!this->is_in_map(this->goal_pose[0], this->goal_pose[1])) {
        std::cout << "no map" << std::endl;
        return nullptr;
    }

    const float THETA_RESOLUTION = M_PI / 18.0;

    // std::vector<std::vector<bool>> visited_node(this->height, std::vector<bool>(this->width, false));
    
    // Lambda comparator for ordering nodes by total cost.
    auto comp = [](const std::shared_ptr<Node>& a, const std::shared_ptr<Node>& b) { 
        return a->f_cost() > b->f_cost(); 
    };
    std::priority_queue<std::shared_ptr<Node>, std::vector<std::shared_ptr<Node>>, decltype(comp)> open_list(comp);
    
    auto start_node = std::make_shared<Node>(Node{
        this->start_pose[0], 
        this->start_pose[1],
        this->start_pose[2],
        0,
        this->heuristic(this->start_pose[0], this->start_pose[1], this->start_pose[2]), 
        nullptr,
        {0., 0.}
    });
    
    open_list.push(start_node);

    std::map<std::tuple<int, int, int>, bool> visited;

    while (!open_list.empty())
    // for(size_t i = 0; i < 1000; ++i)
    {
        std::shared_ptr<Node> current_node = open_list.top();
        open_list.pop();

        if (std::sqrt(std::pow(current_node->x - this->goal_pose[0], 2) + std::pow(current_node->y - this->goal_pose[1], 2)) < this->robot_radius/4 &&
            std::abs(normalize_angle(current_node->theta - this->goal_pose[2])) < M_PI / 9.0)
        {
            // std::cout << "Path found" << std::endl;
            return current_node;
        }

        // int x_disc = static_cast<int>(current_node->x / (2*map_resolution));
        // int y_disc = static_cast<int>(current_node->y / (2*map_resolution));
        // int theta_disc = static_cast<int>(1 * current_node->theta / THETA_RESOLUTION);
        
        // // std::cout << "xN: " << current_node->x << "yN: " << current_node->y << "tN: " << current_node->theta <<std::endl;
        // // std::cout << "xN: " << x_disc << "yN: " << y_disc << "tN: " << theta_disc <<std::endl;
        // // std::cout << "******************************" << std::endl;

        // std::tuple<int, int, int> key = std::make_tuple(x_disc, y_disc, theta_disc);
        // // std::tuple<int, int, float> key = std::make_tuple(x_disc, y_disc, current_node->theta);
        // if (visited[key] or current_node->h_cost > 1e+10) {
        //     continue;
        // }
        // visited[key] = true;

        int add_cost = 0;


        for (const auto& input : this->inputs)
        {
            float v = input.first;
            float w = input.second;

            auto neighbour = get_neighbour(current_node, v, w);
            
            // if (is_collision(neighbour->x, neighbour->y)) {
            //     continue;
            // }

            if(!neighbour) {
                continue;
            }

            int x_disc = static_cast<int>(neighbour->x / (2*map_resolution));
            int y_disc = static_cast<int>(neighbour->y / (2*map_resolution));
            int theta_disc = static_cast<int>(1 * neighbour->theta / THETA_RESOLUTION);
            
            // std::cout << "xN: " << current_node->x << "yN: " << current_node->y << "tN: " << current_node->theta <<std::endl;
            // std::cout << "xN: " << x_disc << "yN: " << y_disc << "tN: " << theta_disc <<std::endl;
            // std::cout << "******************************" << std::endl;

            std::tuple<int, int, int> key = std::make_tuple(x_disc, y_disc, theta_disc);
            // std::tuple<int, int, float> key = std::make_tuple(x_disc, y_disc, current_node->theta);
            if (visited[key] or neighbour->h_cost > 1e+10) {
                continue;
            }
            visited[key] = true;

            // std::cout << "Neighbour: " << add_cost << std::endl;
            // std::cout << "G-Cost: " << neighbour->g_cost << "H-Cost: " << neighbour->h_cost << std::endl;

            // neighbour->g_cost = neighbour->g_cost + this->additional_costs[add_cost];

            open_list.push(neighbour);
            add_cost += 1;

        }
    }

    return nullptr;
}

std::vector<std::tuple<float, float, float>> HybridAStarPlanner::planPath()
{
    std::vector<std::tuple<float, float, float>> path_poses;
    this->goal_node = findPath();

    if (!this->goal_node) {
        std::cerr << "No path found" << std::endl;
        return path_poses;
    }

    auto current = this->goal_node;
    while (current) {
        path_poses.push_back(std::make_tuple(current->x, current->y, current->theta));
        current = current->parent;
    }

    std::reverse(path_poses.begin(), path_poses.end());
    return path_poses;
}

std::vector<std::pair<double, double>> HybridAStarPlanner::get_control_inputs()
{
    std::vector<std::pair<double, double>> control_inputs;
    
    if (!this->goal_node) {
        return control_inputs;
    }
    
    std::vector<std::shared_ptr<Node>> path_nodes;
    auto current = this->goal_node;
    
    while (current && current->parent) {
        path_nodes.push_back(current);
        current = current->parent;
    }
    
    std::reverse(path_nodes.begin(), path_nodes.end());
    
    for (const auto& node : path_nodes) {
        control_inputs.push_back(node->parent_input);
    }

    control_inputs.push_back(control_inputs.back());
    
    return control_inputs;
}