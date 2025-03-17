#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include <opencv2/opencv.hpp>

class OpenCVNode : public rclcpp::Node
{
    public:
        OpenCVNode() : rclcpp::Node("OpenCVNode")
        {
            subscriber_ = create_subscription<sensor_msgs::msg::Image>("/depth_camera/color/image_raw",
                        rclcpp::SystemDefaultsQoS(),
                        std::bind(&OpenCVNode::ImageCallback,
                        this, std::placeholders::_1));
        }
    private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscriber_;
    
    void ImageCallback(const sensor_msgs::msg::Image::SharedPtr msg) const
    {
        try
        {
            cv::Mat frame = cv_bridge::toCvShare(msg, "32FC1")->image;
            cv::imshow("Camera", frame);
            cv::waitKey(10);
        }
        catch (cv_bridge::Exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }

};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OpenCVNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}