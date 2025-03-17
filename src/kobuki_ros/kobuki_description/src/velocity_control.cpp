#include <memory>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist_stamped.hpp>

using namespace std::chrono_literals;

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node =
    std::make_shared<rclcpp::Node>("diff_drive_test_node");

  auto publisher = node->create_publisher<geometry_msgs::msg::TwistStamped>(
    "/diff_drive_base_controller/cmd_vel", 10);

  RCLCPP_INFO(node->get_logger(), "node created");

  geometry_msgs::msg::TwistStamped command;

  command.twist.linear.x = 0.3;
  command.twist.linear.y = 0.0;
  command.twist.linear.z = 0.0;

  command.twist.angular.x = 0.0;
  command.twist.angular.y = 0.0;
  command.twist.angular.z = 0.5;

  while (1) {
    command.header.stamp = node->now();
    publisher->publish(command);
    std::this_thread::sleep_for(50ms);
    rclcpp::spin_some(node);
  }
  rclcpp::shutdown();

  return 0;
}
