#include "kobuki_planner/map.hpp"

MapHandler::MapHandler(const std::string & node_name, const bool map_from_pgm, const std::string & pgm_path, int pixel_safety, std::string planner_type)
: Node(node_name), pixel_safety(pixel_safety)
{
    // this->pixel_safety = pixel_safety;
    this-> planner_type = planner_type;

    if (map_from_pgm) {
        this->readPGM(pgm_path);
        RCLCPP_INFO(this->get_logger(), "Map read from PGM");
    } else {
        // Map topic
        this->map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "map", 10, std::bind(&MapHandler::map_callback, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "...");
    }

    this->tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    this->tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*this->tf_buffer_);

    // Goal pose topic
    this->goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "goal_pose", 10, std::bind(&MapHandler::goal_callback, this, std::placeholders::_1));

    // Start pose topic
    this->start_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/diff_drive_base_controller/odom", 10, std::bind(&MapHandler::position_callback, this, std::placeholders::_1));

    // Timer
    this->pub_path_             = this->create_publisher<nav_msgs::msg::Path>("plan", 10);
    this->pub_posevelvector_    = this->create_publisher<kobuki_msgs::msg::PoseVelVector>("posevelvector", 10);
    // this->timer_                = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&MapHandler::Planner, this));

    this->timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&MapHandler::PeriodicPathPublish, this)
    );

    RCLCPP_INFO(this->get_logger(), "MapHandler initialization complete");
}

void MapHandler::map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{

    this->origin = {msg->info.origin.position.x, msg->info.origin.position.y};
    this->width  = msg->info.width;
    this->height = msg->info.height;
    this->map_resolution = msg->info.resolution;

    // Map resize
    this->map.resize(this->height);
    for (int i = 0; i < this->height; i++) {
        this->map[i].resize(this->width);
    }

    for (int y = 0; y < this->height; y++) {
        for (int x = 0; x < this->width; x++) {
        //   int index = y * this->width + x;
        int index = (this->height - 1 - y) * this->width + x;
        if (index < static_cast<int>(msg->data.size())) {
            int value = msg->data[index];
            if (value == -1) {
                this->map[y][x] = 205;
            } else {
                this->map[y][x] = static_cast<unsigned char>(255 - (value * 255 / 100));
            }
        }
        }
    }

//   cv::Mat image(this->height, this->width, CV_8UC1);
//     for (int y = 0; y < this->height; y++) {
//         for (int x = 0; x < this->width; x++) {
//             image.at<uchar>(y, x) = this->map[y][x];
//         }
//     }

//     cv::Mat resized;
//     cv::resize(image, resized, cv::Size(), 2.0, 2.0, cv::INTER_NEAREST);

//     cv::imshow("Map Visualization", resized);
//     cv::waitKey(1);

    if (!this->map_gotten) {
        RCLCPP_INFO(this->get_logger(), "Received map data. Width: %d, Height: %d", this->width, this->height);
        RCLCPP_INFO(this->get_logger(), "Map processed and stored internally");
        this->map_gotten = true;
    }

}

void MapHandler::position_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    // tf2::Quaternion q(
    //     msg->pose.pose.orientation.x,
    //     msg->pose.pose.orientation.y,
    //     msg->pose.pose.orientation.z,
    //     msg->pose.pose.orientation.w
    // );
    // double roll, pitch, yaw;
    // tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    // this->start_meters = {msg->pose.pose.position.x, msg->pose.pose.position.y, yaw};
    geometry_msgs::msg::PoseStamped pose_in, pose_out;
    pose_in.header = msg->header;
    pose_in.pose = msg->pose.pose;

    try {
        pose_out = tf_buffer_->transform(pose_in, "map", tf2::durationFromSec(0.2));
        tf2::Quaternion q(
            pose_out.pose.orientation.x,
            pose_out.pose.orientation.y,
            pose_out.pose.orientation.z,
            pose_out.pose.orientation.w
        );
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
        this->start_meters = {pose_out.pose.position.x, pose_out.pose.position.y, yaw};
    } catch (tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(), "Could not transform start pose to map: %s", ex.what());
    }

    // RCLCPP_INFO(this->get_logger(), "Planner start pose (map): x=%.3f, y=%.3f, th=%.3f",
    // this->start_meters[0], this->start_meters[1], this->start_meters[2]);
}

void MapHandler::goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{

    // tf2::Quaternion q(
    //     msg->pose.orientation.x,
    //     msg->pose.orientation.y,
    //     msg->pose.orientation.z,
    //     msg->pose.orientation.w
    // );
    // double roll, pitch, yaw;
    // tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    // this->goal_meters = {msg->pose.position.x, msg->pose.position.y, yaw};

    // this->is_first = true;

    // this->Planner();

    geometry_msgs::msg::PoseStamped pose_in = *msg, pose_out;
    try {
        pose_out = tf_buffer_->transform(pose_in, "map", tf2::durationFromSec(0.2));
        tf2::Quaternion q(
            pose_out.pose.orientation.x,
            pose_out.pose.orientation.y,
            pose_out.pose.orientation.z,
            pose_out.pose.orientation.w
        );
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

        this->goal_meters = {pose_out.pose.position.x, pose_out.pose.position.y, yaw};
        this->is_first = true;
        this->Planner();
    } catch (tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(), "Could not transform goal pose to map: %s", ex.what());
    }

    // RCLCPP_INFO(this->get_logger(), "Planner goal pose (map): x=%.3f, y=%.3f, th=%.3f",
    // this->goal_meters[0], this->goal_meters[1], this->goal_meters[2]);

}

void MapHandler::readPGM(const std::string& path_pgm)
{
    // Open the PGM image
    std::ifstream path(path_pgm, std::ios::binary);
    if (!path.is_open()){
        RCLCPP_ERROR(this->get_logger(), "Cannot open the PGM file");
    };

    // Verify if PGM format is P5 or P2
    std::string format;
    path >> format;
    if (format != "P5" && format != "P2") {
        throw std::runtime_error("Invalid PGM format. Expected P5 or P2, got: " + format);
    }

    // Read header (map size)
    char c;
    path >> c;
    while (c == '#') {
        path.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        path >> c;
    }
    path.unget();

    path >> this->width >> this->height;
    int depth;
    path >> depth;
    path.get();

    // Map resize
    this->map.resize(this->height);
    for (int i = 0; i < this->height; i++) {
        this->map[i].resize(this->width);
    }

    // Read data
    if (format == "P5") {  // P5
        for (int y = 0; y < this->height; y++) {
        for (int x = 0; x < this->width; x++) {
            path.read(reinterpret_cast<char*>(&map[y][x]), 1);
        }
        }
    } else {  // P2
        for (int y = 0; y < this->height; y++) {
        for (int x = 0; x < this->width; x++) {
            int val;
            path >> val;
            this->map[y][x] = static_cast<unsigned char>(val);
        }
        }
    }

    this->map_resolution = 0.05;

    RCLCPP_INFO(this->get_logger(), "Successfully read PGM file: %s", path_pgm.c_str());

    this->Planner();

}

void MapHandler::savePGM(std::vector<std::vector<unsigned char>> new_map, const std::string& name)
{

    std::ofstream out(name, std::ios::binary);
    out << "P5\n" << this->width << " " << this->height << "\n255\n";

    for (int i = 0; i < this->height; ++i)
        for (int j = 0; j < this->width; ++j)
            out.put(static_cast<char>(new_map[i][j]));
}

void MapHandler::addRoute(std::vector<std::vector<unsigned char>>& new_map, std::vector<std::tuple<float, float, float>> route) {

    for (size_t i = 0; i < route.size(); ++i) {
        auto [meter_x, meter_y, theta] = route[i];
        std::vector<int> coord_map = MtoP({meter_x, meter_y}, this->origin, this->height, this->map_resolution);
        int x = coord_map[0];
        int y = coord_map[1];
        new_map[y][x] = 127;
    }
}

void MapHandler::Planner()
{
    if (this->map_gotten)
    {
        if (this->goal_meters.empty())
        {
            this->goal_meters = {2., 2.5, M_PI/2};
        }

        if (this->start_meters.empty())
        {
            this->start_meters = {0., 0., 0.};
        }

        // Add planners
        auto t_start = std::chrono::steady_clock::now();
        std::vector<std::tuple<float, float, float>> route;
        std::vector<std::pair<double, double>>       inputs;

        HybridAStarPlanner planner(this->map, this->map_resolution, 0.20, this->origin, this->start_meters, this->goal_meters);
        route = planner.planPath();

        auto t_end = std::chrono::steady_clock::now();
        auto duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(t_end - t_start).count();
        RCLCPP_INFO(this->get_logger(), "Hybrid A* Path planning took %ld ms", duration_ms);

        t_start = std::chrono::steady_clock::now();
        std::vector<std::pair<int, int>> route1;

        AStarPlanner planner1(this->map, this->map_resolution, 4, this->origin, this->start_meters, this->goal_meters);
        route1 = planner1.planPath();

        t_end = std::chrono::steady_clock::now();

        duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(t_end - t_start).count();
        RCLCPP_INFO(this->get_logger(), "A* Path planning took %ld ms", duration_ms);

        if (route1.empty()) {
            RCLCPP_INFO(this->get_logger(), "No A* path found");
        }else{
            auto new_map_explored1 = this->map;
            for (const auto& coords : planner1.explored_nodes_coords) { //
                int px = coords[0];
                int py = coords[1];
                if (px >= 0 && px < this->width && py >= 0 && py < this->height) {
                    new_map_explored1[py][px] = 180;
                }
            }
            this->addRoute1(new_map_explored1, route1);
            this->savePGM1(new_map_explored1, "explored_nodes_A_star.pgm");
        }

        if(!planner.goal_in_map)
        {
            RCLCPP_WARN(this->get_logger(), "Goal not in map");
            return;
        }

        if(planner.goal_collision)
        {
            RCLCPP_WARN(this->get_logger(), "Goal in collision");
            return;
        }

        inputs = planner.get_control_inputs(route);

        // this->addRoute(route);
        if (route.empty()) {
            RCLCPP_INFO(this->get_logger(), "No path found");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Path found");

        this->last_route_ = route;
        this->last_inputs_ = inputs;
        // auto new_map = this->map;
        // this->addRoute(new_map, route);
        // this->savePGM(new_map, "route.pgm");
        this->PubPath(route, inputs);

        auto new_map_explored = this->map;
        for (const auto& coords : planner.explored_nodes_coords) { //
            int px = coords[0];
            int py = coords[1];
            if (px >= 0 && px < this->width && py >= 0 && py < this->height) {
                new_map_explored[py][px] = 180;
            }
        }
        this->addRoute(new_map_explored, route);
        this->savePGM(new_map_explored, "explored_nodes.pgm");
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "No map received. Wait ...");
    }
}

void MapHandler::PubPath(std::vector<std::tuple<float, float, float>> route, std::vector<std::pair<double, double>> inputs)
{

    kobuki_msgs::msg::PoseVel path_vel_point;

    this->path_to_pub.poses.clear();
    this->posevel_to_pub.points.clear();

    this->path_to_pub.header.frame_id = "map";
    this->path_to_pub.header.stamp    = this->get_clock()->now();

    for (size_t i = 0; i < route.size(); ++i) {
        auto [x, y, theta] = route[i];
        auto [v, w]        = inputs[i];

        this->point_to_push.pose.position.x = x;
        this->point_to_push.pose.position.y = y;
        this->point_to_push.pose.position.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0, 0, theta);  // theta en radianes
        this->point_to_push.pose.orientation.x = q.x();
        this->point_to_push.pose.orientation.y = q.y();
        this->point_to_push.pose.orientation.z = q.z();
        this->point_to_push.pose.orientation.w = q.w();

        path_vel_point.x      = x;
        path_vel_point.y      = y;
        path_vel_point.theta  = theta;
        path_vel_point.v      = v;
        path_vel_point.w      = w;

        // std::cout << "x: " << x << " y: " << y << " theta: " << theta << " v: " << v << " w: " << w << std::endl;

        this->path_to_pub.poses.push_back(this->point_to_push);
        this->posevel_to_pub.points.push_back(path_vel_point);
    }

    posevel_to_pub.num_points = this->posevel_to_pub.points.size();

    this->pub_path_->publish(this->path_to_pub);
    if (this->is_first){
        this->pub_posevelvector_->publish(this->posevel_to_pub);
        RCLCPP_INFO(this->get_logger(), "Path published");
    }
    this->is_first = false;
}

void MapHandler::PeriodicPathPublish()
{
    if (!this->last_route_.empty() && !this->last_inputs_.empty()) {
        this->PubPath(this->last_route_, this->last_inputs_);
    }
}

void MapHandler::addRoute1(std::vector<std::vector<unsigned char>>& new_map, std::vector<std::pair<int, int>> route) {
    for (size_t i = 0; i < route.size(); ++i) {
        int x = route[i].first;
        int y = route[i].second;
        if (y >= 0 && y < static_cast<int>(new_map.size()) && x >= 0 && x < static_cast<int>(new_map[0].size())) {
            new_map[y][x] = 127;
        }
    }
}

void MapHandler::savePGM1(const std::vector<std::vector<unsigned char>>& new_map, const std::string& name) {
    std::ofstream out(name, std::ios::binary);
    out << "P5\n" << new_map[0].size() << " " << new_map.size() << "\n255\n";
    for (int i = 0; i < static_cast<int>(new_map.size()); ++i)
        for (int j = 0; j < static_cast<int>(new_map[0].size()); ++j)
            out.put(static_cast<char>(new_map[i][j]));
}