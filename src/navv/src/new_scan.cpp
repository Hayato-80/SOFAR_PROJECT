#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include "rclcpp/qos.hpp" // Include QoS header

class ScanFrameChanger : public rclcpp::Node
{
public:
    ScanFrameChanger()
        : Node("scan_frame_changer")
    {
        rclcpp::QoS qos(rclcpp::KeepLast(10));
        qos.best_effort();

        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/waffle2/scan",
            qos, // Use the custom QoS profile
            std::bind(&ScanFrameChanger::scan_callback, this, std::placeholders::_1));
            
        publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/new_scan", 10);
        new_frame_id_ = "my_new_scan_frame";
    }

private:
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        sensor_msgs::msg::LaserScan new_msg = *msg;
        new_msg.header.frame_id = new_frame_id_;
        publisher_->publish(new_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
    std::string new_frame_id_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ScanFrameChanger>());
    rclcpp::shutdown();
    return 0;
}