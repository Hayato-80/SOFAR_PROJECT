#include "rclcpp/rclcpp.hpp"
#include "turtlesim/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <unordered_map>
#include <cmath>
#include <vector>
#include <memory>
#include <my_interfaces/srv/get_obstacle_pose.hpp>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/static_transform_broadcaster.h"

using namespace std::chrono_literals;

namespace my_pkg
{

class Pose_Server : public rclcpp::Node
{
public:
    Pose_Server(rclcpp::NodeOptions options) : Node("get_obstacle_pose", options)
    {
        obstacle_positions_ = {
            {"obstacle1", {1.0, 2.0, 0.0}},
            {"obstacle2", {3.5, 4.0, 0.0}}
        };

        turtle_pose_service_ = this->create_service<my_interfaces::srv::GetObstaclePose>(
            "get_pose", std::bind(&Pose_Server::get_poses, this, std::placeholders::_1, std::placeholders::_2));
        
    }
  
private:
    rclcpp::Service<my_interfaces::srv::GetObstaclePose>::SharedPtr obsstacles_pose_service_;
    std::unordered_map<std::string, turtlesim::msg::Pose> obsstacle_positions_;
    
    void get_turtle_poses(std::shared_ptr<my_interfaces::srv::GetObstaclePose::Request> request,
        std::shared_ptr<my_interfaces::srv::GetObstaclePose::Response> response)
    {
        
        const auto& pose = obstacle_poses_;
        response->x = pose.x;
        response->y = pose.y;
        response->theta = pose.theta;
        
    }
};

}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  rclcpp::spin(std::make_shared<my_pkg::Pose_Server>(options));
  rclcpp::shutdown();
  return 0;
}
