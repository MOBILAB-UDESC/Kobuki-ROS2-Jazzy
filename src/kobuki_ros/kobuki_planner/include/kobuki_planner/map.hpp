#include <iostream>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>
#include <fstream>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <opencv2/opencv.hpp>
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include "kobuki_planner/HybridAstar.hpp"

#include "kobuki_msgs/msg/pose_vel.hpp"
#include "kobuki_msgs/msg/pose_vel_vector.hpp"

class MapHandler : public rclcpp::Node
{
    public:
        //Constructor
        MapHandler(const std::string & node_name,
                   const bool map_from_pgm,
                   const std::string & pgm_path = "",
                   int pixel_safety = 0,
                   std::string planner_type = "Astar");

    private:

        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

        // Subscribers
        rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr start_sub_;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;
        rclcpp::Publisher<kobuki_msgs::msg::PoseVelVector>::SharedPtr pub_posevelvector_;
        rclcpp::TimerBase::SharedPtr timer_;
        
        // Map data
        std::vector<std::vector<unsigned char>> map;
        int width, height;
        double map_resolution;
        std::vector<double> origin = {-9.05697, -9.05745};
        int pixel_safety;
        bool map_gotten = false;

        // Callback functions
        void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
        void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
        void position_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

        // PGM file handling
        void readPGM(const std::string& path_pgm);
        void savePGM(std::vector<std::vector<unsigned char>> new_map, const std::string& name);
        void addRoute(std::vector<std::vector<unsigned char>>& new_map, std::vector<std::tuple<float, float, float>> route);

        // Path planning
        std::string planner_type;
        std::vector<int> start_pixels   = {};
        std::vector<int> goal_pixels    = {};
        std::vector<double> start_meters = {};
        std::vector<double> goal_meters  = {};

        nav_msgs::msg::Path             path_to_pub;
        kobuki_msgs::msg::PoseVelVector posevel_to_pub;
        geometry_msgs::msg::PoseStamped point_to_push;
        std::vector<std::tuple<float, float, float>> last_route_;
        std::vector<std::pair<double, double>> last_inputs_;
        bool is_first = true;
        void PeriodicPathPublish();
        void Planner();
        // void PubPath(std::vector<std::pair<float, float>> route);
        void PubPath(std::vector<std::tuple<float, float, float>> route, std::vector<std::pair<double, double>> inputs);

        void addRoute1(std::vector<std::vector<unsigned char>>& new_map, std::vector<std::pair<int, int>> route);

        void savePGM1(const std::vector<std::vector<unsigned char>>& new_map, const std::string& name);

};