#include <vector>
#include <queue>
#include <memory>
#include <cmath>
#include <algorithm>
#include <iostream>

#include "kobuki_planner/utils.hpp"  // MtoP, PtoM, normalize_angle
class AStarPlanner
{
    public:

        AStarPlanner(std::vector<std::vector<unsigned char>> map,
                    double map_resolution,
                    int pixel_safety,
                    std::vector<double> origin = {-9.05745, -9.05697},
                    std::vector<double> start_meters= {0, 0},
                    std::vector<double> goal_meters= {0, 0});
        
        struct Node {
            int x, y;                                           // Position in map
            float g_cost;                                       // Cost from the start node to the current node
            float h_cost;                                       // Estimated cost from the current node to the goal node
            std::shared_ptr<Node> parent;                       // Save the parent of a node
        
            float f_cost() const { return g_cost + h_cost; }    // Total cost from the start node to the goal node
        
            bool operator>(const Node& other) const {
                return this->f_cost() > other.f_cost();
            }
        };

        std::vector<std::vector<int>> explored_nodes_coords;

    // private:

        // Map dim
        std::vector<std::vector<unsigned char>> map;
        int   width, height;
        int   pixel_safety;
        float map_resolution;
        std::vector<float> origin;

        // Vectors for start and goal points
        std::vector<int> goal;
        std::vector<int> start;

        // W, NW, N, NE, E, SE, S, NS
        const int dx_[8]     = {-1,    -1,  0,     1,  1,     1,  0,    -1};
        const int dy_[8]     = { 0,     1,  1,     1,  0,    -1, -1,    -1};
        const float costs[8] = {1., 1.414, 1., 1.414, 1., 1.414, 1., 1.414};
        // W, N, E, S
        // const int dx_[4]     = {-1,  0,  1,  0};
        // const int dy_[4]     = { 0,  1,  0, -1};
        // const float costs[4] = {1., 1., 1., 1.};

        // Utils
        bool valid_point(int x, int y) const;
        bool isFree(int x, int y) const;
        float heuristic(int x, int y) const;
        std::vector<std::vector<unsigned char>> new_map();

        // Path planning
        std::shared_ptr<Node> findPath();
        std::vector<std::pair<int, int>> planPath();

};