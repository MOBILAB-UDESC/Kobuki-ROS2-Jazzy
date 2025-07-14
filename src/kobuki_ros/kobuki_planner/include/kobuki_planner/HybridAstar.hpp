#include <vector>
#include <queue>
#include <memory>
#include <cmath>
#include <algorithm>
#include <iostream>
#include <map>
#include <unordered_map>
#include <chrono>

#include "kobuki_planner/utils.hpp"  // MtoP, PtoM, normalize_angle
#include "kobuki_planner/Astar.hpp"  // A* planner for heuristic precomputation
#include "kobuki_planner/smooth.hpp"

extern "C" {
    #include "dubins.h"
}

class HybridAStarPlanner {
    public:
        HybridAStarPlanner(std::vector<std::vector<unsigned char>> map,
                    double map_resolution,
                    float robot_radius,
                    std::vector<double> origin = {-9.05745, -9.05697},
                    std::vector<double> start  = {0., 0.},
                    std::vector<double> goal   = {0., 0.});

        std::vector<std::tuple<float, float, float>> planPath();
        std::vector<std::pair<double, double>> get_control_inputs(std::vector<std::tuple<float, float, float>> route);
        bool goal_in_map    = true;
        bool goal_collision = false;

        std::vector<std::vector<int>> explored_nodes_coords;

    private:

        /*******************************************************************************
                                        MAP CONFIGURATION
        *******************************************************************************/

        std::vector<std::vector<unsigned char>> map;
        double map_resolution  = 0.05;
        int width, height;
        std::vector<double> origin;
        
        struct Node {
            double  x, y;                                       // Position on the map (meters)
            double theta;                                       // Orientation on the map (radians)
            float g_cost;                                       // Cost from the start node to this node
            double h_cost;                                      // Estimated cost from this node to the goal node
            std::shared_ptr<Node> parent;                       // Pointer to the parent node (for path reconstruction)
            // Node* parent;                                       // Alternative: raw pointer to the parent node
            std::pair<double, double> parent_input;             // Inputs applied to reach this node

            float f_cost() const { return g_cost + h_cost; }    // Total cost

            // bool operator>(const Node& other) const {           // Operator overloading (Alternative)
            //     return this->f_cost() > other.f_cost();
            // }
        };

        bool is_in_map(double x, double y) const;
        bool is_in_map(int x, int y) const;
        bool is_collision(double x, double y) const;
        bool is_collision(int x, int y) const;
        std::shared_ptr<Node> get_neighbour(std::shared_ptr<Node> current_node, float v, float w);

        /*******************************************************************************
                                       STATE CONFIGURATION
        *******************************************************************************/
        const float DT     = 0.1;          // Sampling time in seconds
        bool iscircular    = true;         // Kobuki is circular?
        float robot_radius = 0.35;         // Kobuki's radius in meters
        float robot_width  = 0.25;         // Kobuki's width in meters
        float robot_length = 0.50;         // Kobuki's length in meters
        
        const float THETA_RESOLUTION = M_PI / 18.0;

        std::vector<double> goal_pose;     // {x, y, theta}
        std::vector<double> start_pose;    // {x, y, theta}

        // Actions for controlling the robot {v, w}
        std::vector<std::pair<double, double>> inputs = {
            {0.5, 0.0},                      // Move forward
            {0.5, M_PI / 1.8},               // Turn right forward
            {0.5, -M_PI / 1.8},              // Turn left forward
            // {-1.0, 0.0},                     // Move backward
            // {-1., M_PI / 1.8},               // Turn right backward
            // {-1., -M_PI / 1.8},              // Turn left backward
            // {0.0, M_PI / 1.80},           // Turn rigth
            // {0.0, -M_PI / 1.80},          // Turn left
        };

        const float reverse_cost = 2.0;  // Cost for reversing
        const float forward_cost = 1.0;  // Cost for moving forward
        const float turn_cost    = 0.25;  // Cost for turning
        const float maps_cost    = 1.0;  // Cost for turning

        /*******************************************************************************
                                        PATH PLANNING
        *******************************************************************************/
        // struct DubinsPath {
        //     std::vector<std::pair<float, float>> points;
        //     std::vector<float> thetas;
        //     float total_cost;
        //     bool valid;
        // };

        struct TupleHash
        {
            std::size_t operator()(const std::tuple<int, int, int>& t) const {
                return std::hash<int>()(std::get<0>(t)) ^ std::hash<int>()(std::get<1>(t)) ^ std::hash<int>()(std::get<2>(t));
            };
        };

        std::vector<std::vector<double>> heuristic_map;
        std::shared_ptr<Node> goal_node;
        DubinsPath final_path;
        double heuristic(double x, double y, double theta) const;
        void precompute_heuristic();
        std::shared_ptr<Node> findPath();
        DubinsPath get_analytic_expansion(std::shared_ptr<Node> current_node);
        DubinsPath get_dubins_path(std::shared_ptr<Node> current_node) const;

};