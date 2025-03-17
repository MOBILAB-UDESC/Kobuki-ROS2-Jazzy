// The ROS2 navigation system provides velocities through the cmd_vel topic using the Twist message type
// However, this project uses the TwistStamped message type. Therefore, remapping the topics is necessary 
// to ensure proper communication.

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

class TwistToTwistStamped : public rclcpp::Node
{
public:
  TwistToTwistStamped()
  : Node("twist_to_twist_stamped")
  {
    // ROS2 navigation velocities
    cmd_vel_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10, std::bind(&TwistToTwistStamped::cmd_vel_callback, this, std::placeholders::_1));

    // Kobuki velocities
    twist_stamped_publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
      "/diff_drive_base_controller/cmd_vel", 10);
  }

private:
  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    auto twist_stamped_msg = std::make_shared<geometry_msgs::msg::TwistStamped>();

    twist_stamped_msg->header.stamp = this->now();
    twist_stamped_msg->header.frame_id = "base_link";

    twist_stamped_msg->twist = *msg;

    twist_stamped_publisher_->publish(*twist_stamped_msg);
  }

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscriber_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_stamped_publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TwistToTwistStamped>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}