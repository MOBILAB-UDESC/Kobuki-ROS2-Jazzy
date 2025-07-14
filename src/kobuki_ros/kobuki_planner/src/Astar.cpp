#include "kobuki_planner/Astar.hpp"

AStarPlanner::AStarPlanner(std::vector<std::vector<unsigned char>> map,
                           double map_resolution,
                           int pixel_safety,
                           std::vector<double> origin_meters,
                           std::vector<double> start_meters,
                           std::vector<double> goal_meters)
{
    this->map            = map;
    this->height         = map.size();
    this->width          = map[0].size();
    this->map_resolution = map_resolution;
    // this->origin         = MtoP(origin_meters, origin_meters, this->height, this->map_resolution);
    // this->map_resolution = map_resolution;
    this->pixel_safety   = pixel_safety;
    std::cout << "PSX: " << start_meters[0] << ", Y: " << start_meters[1] << ", T: " << start_meters[2] << std::endl;
    std::cout << "PGX: " << goal_meters[0] << ", Y: " << goal_meters[1] << ", T: " << goal_meters[2] << std::endl;
    std::cout << "PGX: " << origin_meters[0] << ", Y: " << origin_meters[1] << ", T: " << origin_meters[2] << std::endl;
    this->start          = MtoP(start_meters, origin_meters, this->height, this->map_resolution);
    this->goal           = MtoP(goal_meters, origin_meters, this->height, this->map_resolution);

};

bool AStarPlanner::valid_point(int x, int y) const
{
    return y >= 0 && y < this->height && x >= 0 && x < this->width && isFree(x, y);
}
bool AStarPlanner::isFree(int x, int y) const
{
    return this->map[y][x] > 0;
}

float AStarPlanner::heuristic(int x, int y) const
{
    // Return 0 so it is like Dijkstra's algorithm
    return 0.0f;

    // Manhattan
    return std::abs(x - this->goal[0]) + std::abs(y - this->goal[1]);
    
    // Euclidean
    // return std::sqrt(std::pow(x - this->goal[0], 2) + std::pow(y - this->goal[1], 2));
};

std::shared_ptr<AStarPlanner::Node> AStarPlanner::findPath()
{

    std::cout << "SX: " << this->start[0] << ", Y: " << this->start[1] << std::endl;
    std::cout << "GX: " << this->goal[0] << ", Y: " << this->goal[1] << std::endl;
    std::cout << "map[SX][SY]: " << static_cast<int>(this->map[this->start[1]][this->start[0]]) << std::endl;
    std::cout << "map[GX][GY]: " << static_cast<int>(this->map[this->goal[1]][this->goal[0]]) << std::endl;

    /*
        Astar 
        Return a chain from the initial to the goal node if a path is found.
    */

    // Create a 2DMatrix initialized with false to track visited nodes.
    std::vector<std::vector<bool>> visited_node(this->height, std::vector<bool>(this->width, false));
    
    // Lambda comparator for ordering nodes by total cost.
    auto comp = [](const std::shared_ptr<Node>& a, const std::shared_ptr<Node>& b) { 
        return a->f_cost() > b->f_cost(); 
    };
    std::priority_queue<std::shared_ptr<Node>, std::vector<std::shared_ptr<Node>>, decltype(comp)> open_list(comp);
    
    auto start_node = std::make_shared<Node>(Node{
        this->start[0], 
        this->start[1], 
        0,
        this->heuristic(this->start[0], this->start[1]), 
        nullptr
    });
    
    open_list.push(start_node);

    while(!open_list.empty())
    {
        auto current_node = open_list.top();
        open_list.pop();

        // Check if the goal was achieved.
        if (current_node->x == this->goal[0] && current_node->y == this->goal[1]) {
            return current_node;
        }

        // Skip the iteration if the node was already visited.
        if (visited_node[current_node->x][current_node->y])
            continue;

        visited_node[current_node->x][current_node->y] = true;

        for (int i = 0; i < sizeof(this->dx_); ++i) {
            int nx = current_node->x + this->dx_[i];
            int ny = current_node->y + this->dy_[i];
            
            // Skip the iteration if the node is not valid.
            if (!this->valid_point(nx, ny)) {
                continue;
            }
            
            // Straight moves have unit cost and diag moves cost 1.41...
            float move_cost = (i % 2 == 0) ? 0.05 : std::sqrt(2.0)*0.05;
            // float move_cost = 0.05;
            
            // Add no visited neighbors
            if (!visited_node[nx][ny]) {
                auto neighbor_node = std::make_shared<Node>(
                    Node{nx, ny, 
                        current_node->g_cost + move_cost,
                        heuristic(nx, ny), 
                        current_node}
                );
                open_list.push(neighbor_node);
                this->explored_nodes_coords.push_back({nx, ny});
            }
        }
    }

    return nullptr;
};

std::vector<std::vector<unsigned char>> AStarPlanner::new_map()
{

    /*
        Add thickness to each pixel
        Return a new map
    */

    auto n_map = this->map;

    for (int i = 0; i < this->height; ++i) {
        for (int j = 0; j < this->width; ++j) {
            if (this->map[i][j] == 0) {
                for (int dx = -this->pixel_safety; dx <= this->pixel_safety; ++dx) {       // Add pixelsafety pixels above and below
                    for (int dy = -this->pixel_safety; dy <= this->pixel_safety; ++dy) {   // Add pixelsafety pixels to the left and right
                        int ni = i + dy;
                        int nj = j + dx;
                        if (this->valid_point(nj, ni)) {
                            n_map[ni][nj] = 0;
                        }
                    }
                }
            }
        }
    }
    return n_map;
}

std::vector<std::pair<int, int>> AStarPlanner::planPath()
{
    std::vector<std::pair<int, int>> path;
    this->map = this->new_map();
    auto goal_node = findPath();
    
    if (!goal_node) {
        std::cout << "no path" << std::endl;
        return path;
    }
    
    auto current = goal_node;
    while (current) {
        path.push_back({current->x, current->y});
        current = current->parent;
    }
    
    std::reverse(path.begin(), path.end());

    return path;
};