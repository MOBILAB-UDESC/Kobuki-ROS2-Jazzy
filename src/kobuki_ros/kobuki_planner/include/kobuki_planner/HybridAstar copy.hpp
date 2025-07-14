#include <vector>
#include <queue>
#include <memory>
#include <cmath>
#include <algorithm>
#include <iostream>
#include <map>

#include "kobuki_planner/utils.hpp"  // MtoP, PtoM, normalize_angle
#include "kobuki_planner/Astar.hpp"  // A* planner for heuristic precomputation

class HybridAStarPlanner {
    public:
        HybridAStarPlanner(std::vector<std::vector<unsigned char>> map,
                    double map_resolution,
                    float robot_radius,
                    std::vector<double> origin = {-9.05745, -9.05697},
                    std::vector<double> start  = {0., 0.},
                    std::vector<double> goal   = {0., 0.});

        std::vector<std::tuple<float, float, float>> planPath();
        std::vector<std::pair<double, double>> get_control_inputs();

    private:

        /*******************************************************************************
                                        MAP CONFIGURATION
        *******************************************************************************/

        std::vector<std::vector<unsigned char>> map;
        double map_resolution  = 0.05;
        int width, height;
        std::vector<double> origin;
        
        struct Node {
            double  x, y;                                       // Position in map
            double theta;                                       // Orientation in map
            float g_cost;                                       // Cost from the start node to the current node
            double h_cost;                                      // Estimated cost from the current node to the goal node
            std::shared_ptr<Node> parent;                       // Save the parent of this node
            std::pair<double, double> parent_input;             // Inputs of this node

            float f_cost() const { return g_cost + h_cost; }    // Total cost from the start node to the goal node

            bool operator>(const Node& other) const {           // Priority queue comparator
                return this->f_cost() > other.f_cost();
            }
        };

        bool is_in_map(double x, double y) const;
        bool is_collision(double x, double y) const;
        bool is_collision(int x, int y) const;
        std::shared_ptr<HybridAStarPlanner::Node> get_neighbour(std::shared_ptr<HybridAStarPlanner::Node> current_node, float v, float w);

        /*******************************************************************************
                                       STATE CONFIGURATION
        *******************************************************************************/
        const float DT     =  0.1;              // Sampling time in seconds
        float robot_radius = 0.25;              // Kobuki's radius in meters

        std::vector<double> goal_pose;          // {x, y, theta}
        std::vector<double> start_pose;         // {x, y, theta}

        // Actions for controlling the robot {v, w}
        std::vector<std::pair<double, double>> inputs = {
            {1.0, 0.0},                      // Move forward
            // {1.0, M_PI / 1.8},               // Turn right forward
            // {1.0, -M_PI / 1.8},              // Turn left forward
            {-1.0, 0.0},                     // Move backward
            {-1.0, M_PI / 1.8},               // Turn right backward
            {-1.0, -M_PI / 1.8},              // Turn left backward
            // {0.0, M_PI / 1.80},           // Turn rigth
            // {0.0, -M_PI / 1.80},          // Turn left
        };

        const float reverse_cost = 2.0;  // Cost for reversing
        const float forward_cost = 1.0;  // Cost for moving forward
        const float turn_cost    = 0.5;  // Cost for turning
        const float maps_cost    = 1.0;  // Cost for turning

        std::vector<float> additional_costs = {
            0.,
            0.,
            0.,
            0.,
            0.,
            0.,
            0.,
            0.
        };

        /*******************************************************************************
                                        PATH PLANNING
        *******************************************************************************/
        
        std::vector<std::vector<double>> heuristic_map;
        std::shared_ptr<Node> goal_node;
        double heuristic(double x, double y, double theta) const;
        void precompute_heuristic();
        std::shared_ptr<Node> findPath();

};