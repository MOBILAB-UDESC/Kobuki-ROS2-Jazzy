#include <iostream>
#include "kobuki_planner/map.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    bool map_from_pgm           = false;
    std::string pgm_path        = "";
    int pixel_safety            = 0;
    std::string planner_type    = "Astar";
    
    if (argc > 1)
    {
        map_from_pgm = true;
        pgm_path = argv[1];
        if (pgm_path.empty())
        {
            map_from_pgm = false;
        }
        try
        {
            pixel_safety = static_cast<int>(std::stof(argv[2])/0.05);
        }
        catch (...)
        {
            std::cerr << "The second arg must be a number. Setting pixel_safety to 0" << std::endl;
            pixel_safety = 0;
        }
        try
        {
            planner_type    = argv[3];
        }
        catch (...)
        {
            std::cerr << "There's no third arg (must be the planner_type)" << std::endl;
        }
        std::cout << "Using PGM file: " << pgm_path << std::endl;
    }
    else
    {
        std::cout << "No PGM file specified. Will wait for map from ROS topic." << std::endl;
    }
    
    auto node = std::make_shared<MapHandler>("map_handler_node", map_from_pgm, pgm_path, pixel_safety, planner_type);
    
    std::cout << "Running map handler node. Press Ctrl+C to exit." << std::endl;
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    
    return 0;
}