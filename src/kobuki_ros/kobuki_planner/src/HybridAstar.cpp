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

};

bool HybridAStarPlanner::is_in_map(double x, double y) const {
    std::vector<int> map_coords = MtoP({x, y}, this->origin, this->height, this->map_resolution);
    return map_coords[0] >= 0 && map_coords[0] < this->width &&
           map_coords[1] >= 0 && map_coords[1] < this->height;

};

bool HybridAStarPlanner::is_in_map(int x, int y) const {
    return x >= 0 && x < this->width &&
           y >= 0 && y < this->height;

};

bool HybridAStarPlanner::is_collision(double x, double y) const {
    // Kobuki has a circular shape, so we need to check for collisions in a circular area
    // In case of a rectangular robot, set iscircular to false

    std::vector<int> coords_pixels = MtoP({x, y}, this->origin, this->height, this->map_resolution);

    int pixel_safety_x;
    int pixel_safety_y;

    if(this->iscircular)
    {
        pixel_safety_x = static_cast<int>(this->robot_radius / this->map_resolution);
        pixel_safety_y = pixel_safety_x;
    }else{
        pixel_safety_x = static_cast<int>(this->robot_width  / this->map_resolution);
        pixel_safety_y = static_cast<int>(this->robot_length / this->map_resolution);
    }

    for (int dx = -pixel_safety_x; dx <= pixel_safety_x; ++dx) {
        for (int dy = -pixel_safety_y; dy <= pixel_safety_y; ++dy) {
            int px = coords_pixels[0] + dx;
            int py = coords_pixels[1] + dy;

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

    int pixel_safety_x;
    int pixel_safety_y;

    if(this->iscircular)
    {
        pixel_safety_x = static_cast<int>(this->robot_radius / this->map_resolution);
        pixel_safety_y = pixel_safety_x;
    }else{
        pixel_safety_x = static_cast<int>(this->robot_width  / this->map_resolution);
        pixel_safety_y = static_cast<int>(this->robot_length / this->map_resolution);
    }

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
};

double HybridAStarPlanner::heuristic(double x, double y, double theta) const
{
    // Manhattan
    // return 0.8*(std::abs(x - this->goal_pose[0]) + std::abs(y - this->goal_pose[1])) + 0.2*(std::abs(theta - this->goal_pose[2]));
    
    // Euclidean
    // return 0.8*(std::sqrt(std::pow(x - this->goal_pose[0], 2) + std::pow(y - this->goal_pose[1], 2))) + 0.2*(std::abs(theta - this->goal_pose[2]));

    std::vector<int> map_coords = MtoP({x, y}, this->origin, this->height, this->map_resolution);
    int px = map_coords[0];
    int py = map_coords[1];

    if (px < 0 || px >= this->width || py < 0 || py >= this->height) {
        return std::numeric_limits<double>::max();
    }

    return heuristic_map[py][px]; 

};

std::shared_ptr<HybridAStarPlanner::Node> HybridAStarPlanner::get_neighbour(std::shared_ptr<HybridAStarPlanner::Node> current_node, float v, float w)
{

    // Kobuki's dynamics
    double next_x, next_y, next_theta;

    next_x = current_node->x;
    next_y = current_node->y;
    next_theta = current_node->theta;

    int n_steps = 100;

    for (int i = 0; i<n_steps; ++i)
    {
        next_x += v * this->DT / n_steps * std::cos(next_theta);
        next_y += v * this->DT / n_steps * std::sin(next_theta);
        next_theta = normalize_angle(next_theta + w * this->DT / n_steps);
    }
    // next_x = next_x + v * this->DT * std::cos(next_theta);
    // next_y = next_y + v * this->DT * std::sin(next_theta);
    // next_theta = normalize_angle(next_theta + w * this->DT);

    if(this->is_collision(next_x, next_y)) {
        return nullptr;
    }

    std::shared_ptr<Node> next_node = std::make_shared<Node>();

    // Add to Node
    next_node->x     = next_x;
    next_node->y     = next_y;
    next_node->theta = next_theta;
    next_node->parent = current_node;
    next_node->parent_input = {v, w};
    next_node->h_cost = heuristic(next_x, next_y, next_theta)*1;

    double incremental_cost = 0.0;

    double distance_traveled = std::sqrt(std::pow(next_x - current_node->x, 2) + std::pow(next_y - current_node->y, 2));
    incremental_cost += distance_traveled * this->forward_cost;

    if (v < 0) {
        incremental_cost *= this->reverse_cost;
    }

    // double angle_change = std::abs(normalize_angle(next_theta - current_node->theta));
    // incremental_cost += angle_change * this->turn_cost;

    // std::vector<int> next_pixel = MtoP({next_x, next_y}, this->origin, this->height, this->map_resolution);
    // int next_px = next_pixel[0];
    // int next_py = next_pixel[1];

    // if (next_px >= 0 && next_px < this->width && next_py >= 0 && next_py < this->height) {
    //     unsigned char map_cell_value = this->map[next_py][next_px];
    //     double cell_cost = 0.0;
    //     if (map_cell_value > 0) {
    //         cell_cost = static_cast<double>(map_cell_value) / 255.0;
    //     }
        
    //     incremental_cost += distance_traveled * cell_cost * this->maps_cost;
    // } else {
    //     incremental_cost = std::numeric_limits<double>::max(); 
    // }

    next_node->g_cost = current_node->g_cost + incremental_cost;
   
    return next_node;
};

void HybridAStarPlanner::precompute_heuristic()
{
    // Backwards Djikstra's algorithm to precompute the heuristic map of every point in the map
    this->heuristic_map.clear();

    // W, NW, N, NE, E, SE, S, NS
    // const int dx[8]      = {-1,    -1,  0,     1,  1,     1,  0,    -1};
    // const int dy[8]      = { 0,     1,  1,     1,  0,    -1, -1,    -1};
    // // const float costs[8] = {1., 1.414, 1., 1.414, 1., 1.414, 1., 1.414};
    // const float costs[8] = {0.05, 0.0707, 0.05, 0.0707, 0.05, 0.0707, 0.05, 0.0707};
    const int dx[4]      = {-1,  0, 1,  0};
    const int dy[4]      = { 0,  1, 0, -1};
    // const float costs[8] = {1., 1.414, 1., 1.414, 1., 1.414, 1., 1.414};
    const float costs[4] = {0.05, 0.05, 0.05, 0.05};

    // Initialize the heuristic map with maximum values
    this->heuristic_map.assign(this->height, std::vector<double>(this->width, std::numeric_limits<double>::max()));

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

    this->heuristic_map[goal_py][goal_px] = 0.0;
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

        for (int i = 0; i < 4; ++i) {
            int next_px = px + dx[i];
            int next_py = py + dy[i];

            if(!this->is_in_map(next_px, next_py)) {
                continue;
            }

            if(this->is_collision(next_px, next_py)) {
                continue;
            }

            double new_cost = cost + costs[i];// + this->map_resolution*euclidean_distance(next_px, next_py, goal_px, goal_py);

            if (new_cost < this->heuristic_map[next_py][next_px]) {
                this->heuristic_map[next_py][next_px] = new_cost;
                pq.push({next_px, next_py, new_cost});
            }
        }
    }
}

std::shared_ptr<HybridAStarPlanner::Node> HybridAStarPlanner::findPath()
{
    if (!this->is_in_map(this->goal_pose[0], this->goal_pose[1]) || !this->is_in_map(this->start_pose[0], this->start_pose[1])) {
        this->goal_in_map = false;
        return nullptr;
    }

    if (this->is_collision(this->goal_pose[0], this->goal_pose[1]) || this->is_collision(this->start_pose[0], this->start_pose[1])) {
        this->goal_collision = true;
        return nullptr;
    }

    auto t_start = std::chrono::steady_clock::now();
    this->precompute_heuristic();
    auto t_end = std::chrono::steady_clock::now();

    auto duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(t_end - t_start).count();
    std::cout << "A* pre heuristic took " << duration_ms << " ms." << std::endl;
    // const float THETA_RESOLUTION = M_PI / 18.0;
    
    // Lambda comparator for ordering nodes by total cost.
    // comp = [inputs](variables){function body};
    auto comp = [](const std::shared_ptr<Node>& a, const std::shared_ptr<Node>& b) { 
        return a->f_cost() > b->f_cost(); 
    };
    std::priority_queue<std::shared_ptr<Node>, std::vector<std::shared_ptr<Node>>, decltype(comp)> open_list(comp);
    // In case std::shared_ptr is not used
    // std::priority_queue<Node, std::vector<Node>, std::greater<Node>> open_list;
    
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

    // std::map<std::tuple<int, int, int>, bool> visited;
    std::unordered_map<std::tuple<int, int, int>, bool, TupleHash> visited;

    int expansion_count = 0;
    int nodes_count     = 0;
    int expansion_ratio = 500;

    this->explored_nodes_coords.clear();
    std::shared_ptr<Node> aux_node;
    while (!open_list.empty() && nodes_count < 1000000)
    // for(size_t i = 0; i < 1000; ++i)
    {
        std::shared_ptr<Node> current_node = open_list.top();
        aux_node = current_node;
        open_list.pop();

        if (std::sqrt(std::pow(current_node->x - this->goal_pose[0], 2) + std::pow(current_node->y - this->goal_pose[1], 2)) < this->robot_radius/4 &&
            std::abs(normalize_angle(current_node->theta - this->goal_pose[2])) < M_PI / 9.0)
        {
            std::cout << "Total nodes: " << nodes_count << std::endl;
            return current_node;
        }

        // Analitic expansion as in Nav2
        if (expansion_count % expansion_ratio == 0)
        {
            auto analytic_expansion = this->get_analytic_expansion(current_node);
            double length = dubins_path_length(&analytic_expansion);
            
            std::cout << "Expansion count: " << expansion_count << std::endl;
            if(!length) {
                std::cout << "Analytic expansion failed" << std::endl;
            }
            else{
                // this->inputs = {
                //     {0.5, 0.0},                      // Move forward
                //     {0.5, M_PI / 1.8},               // Turn right forward
                //     {0.5, -M_PI / 1.8},              // Turn left forward
                //     // {-0.5, 0.0},                     // Move backward
                //     {-0.5, M_PI / 1.8},               // Turn right backward
                //     {-0.5, -M_PI / 1.8},              // Turn left backward
                //     // {0.0, M_PI / 1.80},           // Turn rigth
                //     // {0.0, -M_PI / 1.80},          // Turn left
                // };
                this->final_path = analytic_expansion;
                std::cout << "Nodes count: " << nodes_count << std::endl;
                return current_node;
            }
        }

        int x_disc = static_cast<int>(current_node->x / (1*map_resolution));
        int y_disc = static_cast<int>(current_node->y / (1*map_resolution));
        int theta_disc = static_cast<int>(1 * current_node->theta / this->THETA_RESOLUTION);

        std::tuple<int, int, int> key = std::make_tuple(x_disc, y_disc, theta_disc);

        if (visited[key] or current_node->h_cost > 1e+10){continue;}
        visited[key] = true;

        nodes_count++;
        expansion_count++;

        // std::cout << "Node: " << nodes_count << std::endl;
        // std::cout << "XNODE: " << current_node->x << ", YNODE: " << current_node->y << ", TNODE: " << current_node->theta << std::endl;
        for (const auto& input : this->inputs)
        {
            float v = input.first;
            float w = input.second;

            auto neighbour = get_neighbour(current_node, v, w);

            if(!neighbour){continue;}

            open_list.push(neighbour);

            std::vector<int> map_coords = MtoP({neighbour->x, neighbour->y}, this->origin, this->height, this->map_resolution);
            // std::cout << "X: " << neighbour->x << ", Y: " << neighbour->y << ", T: " << neighbour->theta << std::endl;
            // std::cout << "PX: " << map_coords[0] << ", PY: " << map_coords[1] << std::endl;
            // std::cout << "G: " << neighbour->g_cost << ", H: " << neighbour->h_cost << std::endl;
            this->explored_nodes_coords.push_back(map_coords);
        }
    }

    return nullptr;
    // return aux_node;
}

DubinsPath HybridAStarPlanner::get_analytic_expansion(std::shared_ptr<Node> current_node)
{
    // Dubins curves
    DubinsPath empty_path;
    DubinsPath path = this->get_dubins_path(current_node);

    for (double t = 0; t < dubins_path_length(&path); t += 0.1) {
        double pos[3]; dubins_path_sample(&path, t, pos);

        if(this->is_collision(pos[0], pos[1])){return empty_path;}
        
        // std::cout << "x1: " << pos[0] << ", y: " << pos[1] << ", theta: " << normalize_angle(pos[2]) << std::endl;
    }

    return path;

}

DubinsPath HybridAStarPlanner::get_dubins_path(std::shared_ptr<Node> current_node) const
{
    DubinsPath path;
    double q0[3];
    q0[0] = current_node->x;
    q0[1] = current_node->y;
    q0[2] = current_node->theta;
    double q1[3];
    q1[0] = this->goal_pose[0];
    q1[1] = this->goal_pose[1];
    q1[2] = this->goal_pose[2];

    int err = 10;
    if(err != 0) {
        err = dubins_path(&path, q0, q1, 0.1, RLR);
    }
    if(err != 0) {
        err = dubins_path(&path, q0, q1, 0.1, LSR);
    }
    if(err != 0) {
        err = dubins_path(&path, q0, q1, 0.1, RSL);
    }
    if(err != 0) {
        err = dubins_path(&path, q0, q1, 0.1, RSR);
    }
    if(err != 0) {
        err = dubins_path(&path, q0, q1, 0.1, LSL);
    }
    if(err != 0) {
        err = dubins_path(&path, q0, q1, 0.1, LRL);
    }

    std::cout << "Dubins path error: " << err << std::endl;

    return path;
}


std::vector<std::tuple<float, float, float>> HybridAStarPlanner::planPath()
{
    std::vector<std::tuple<float, float, float>> path_poses;
    this->goal_node = findPath();

    double length = dubins_path_length(&this->final_path);

    if (!this->goal_node){
        return path_poses;
    }

    auto current = this->goal_node;
    while (current) {
        path_poses.push_back(std::make_tuple(current->x, current->y, current->theta));
        current = current->parent;
    }

    std::reverse(path_poses.begin(), path_poses.end());

    SmoothPath smoother(path_poses, 0.1);
    path_poses = smoother.Darpa_curve();

    if(!length) {  
        return path_poses;
    }

    for (double t = 0.1; t < length; t += 0.1) {
        double pos[3]; dubins_path_sample(&this->final_path, t, pos);
        path_poses.push_back(std::make_tuple(pos[0], pos[1], normalize_angle(pos[2])));
    }

    return path_poses;
}

std::vector<std::pair<double, double>> HybridAStarPlanner::get_control_inputs(std::vector<std::tuple<float, float, float>> route)
{
    // std::vector<std::pair<double, double>> control_inputs;
    // double length = dubins_path_length(&this->final_path);
    // if (!this->goal_node){
    //     return control_inputs;
    // }
    
    // std::vector<std::shared_ptr<Node>> path_nodes;
    // auto current = this->goal_node;
    
    // while (current) {
    //     path_nodes.push_back(current);
    //     current = current->parent;
    // }
    
    // std::reverse(path_nodes.begin(), path_nodes.end());
    
    // for (const auto& node : path_nodes) {
    //     control_inputs.push_back(node->parent_input);
    // }

    // control_inputs.push_back(control_inputs.back());

    // if(!length) {  
    //     return control_inputs;
    // }

    // for (double t = 0.1; t < length; t += 0.1) {
    //     control_inputs.push_back({0.0, 0.0});
    // }

    // return control_inputs;

    std::vector<std::pair<double, double>> control_inputs;
    
    size_t n = route.size();
    std::vector<double> x_ref(n), y_ref(n);
    for (size_t i = 0; i < n; ++i) {
        x_ref[i] = static_cast<double>(std::get<0>(route[i]));
        y_ref[i] = static_cast<double>(std::get<1>(route[i]));
    }

    std::vector<double> dx_ref(n), dy_ref(n);
    std::vector<double> ddx_ref(n), ddy_ref(n);
    std::vector<double> th_ref(n), v_ref(n), w_ref(n);

    // Primera derivada (dx, dy)
    for (size_t i = 1; i < n - 1; ++i) {
        dx_ref[i] = (x_ref[i + 1] - x_ref[i - 1]) / (2 * this->DT);
        dy_ref[i] = (y_ref[i + 1] - y_ref[i - 1]) / (2 * this->DT);
    }
    dx_ref[0] = (x_ref[1] - x_ref[0]) / this->DT;
    dy_ref[0] = (y_ref[1] - y_ref[0]) / this->DT;
    dx_ref[n - 1] = (x_ref[n - 1] - x_ref[n - 2]) / this->DT;
    dy_ref[n - 1] = (y_ref[n - 1] - y_ref[n - 2]) / this->DT;

    // Theta (orientação)
    // th_ref[0] = std::atan2(dy_ref[0], dx_ref[0]);
    // for (size_t i = 1; i < n; ++i) {
    //     th_ref[i] = std::atan2(dy_ref[i], dx_ref[i]);
    //     while (th_ref[i] - th_ref[i - 1] > M_PI) th_ref[i] -= 2 * M_PI;
    //     while (th_ref[i] - th_ref[i - 1] < -M_PI) th_ref[i] += 2 * M_PI;
    // }

    // Segunda derivada (ddx, ddy)
    for (size_t i = 1; i < n - 1; ++i) {
        ddx_ref[i] = (dx_ref[i + 1] - dx_ref[i - 1]) / (2 * this->DT);
        ddy_ref[i] = (dy_ref[i + 1] - dy_ref[i - 1]) / (2 * this->DT);
    }
    ddx_ref[0] = (dx_ref[1] - dx_ref[0]) / this->DT;
    ddy_ref[0] = (dy_ref[1] - dy_ref[0]) / this->DT;
    ddx_ref[n - 1] = (dx_ref[n - 1] - dx_ref[n - 2]) / this->DT;
    ddy_ref[n - 1] = (dy_ref[n - 1] - dy_ref[n - 2]) / this->DT;

    // Velocidades linear (v) e angular (w)
    for (size_t i = 0; i < n; ++i) {
        v_ref[i] = std::sqrt(dx_ref[i] * dx_ref[i] + dy_ref[i] * dy_ref[i]);
        if (v_ref[i] > 0.001) {
            w_ref[i] = (ddy_ref[i] * dx_ref[i] - ddx_ref[i] * dy_ref[i]) /
                       (dx_ref[i] * dx_ref[i] + dy_ref[i] * dy_ref[i]);
        } else {
            w_ref[i] = 0.0;
        }
        control_inputs.emplace_back(v_ref[i], w_ref[i]);
    }

    return control_inputs;

}