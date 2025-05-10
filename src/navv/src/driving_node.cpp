#include <rclcpp/rclcpp.hpp>
#include <algorithm>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

using namespace std::chrono_literals;

namespace navv
{

    class DrivingNode : public rclcpp::Node
    {
    public:
        DrivingNode(rclcpp::NodeOptions options) : Node("driving_node", options),
                                                   tf_buffer(this->get_clock(), std::chrono::seconds(10), std::shared_ptr<rclcpp::Node>())
        {
            // Declare parameters with default values
            this->declare_parameter("speed", 0.3);
            this->declare_parameter("delta_t", 0.5);

            // Retrieve parameter values
            speed = this->get_parameter("speed").as_double();
            delta_t = this->get_parameter("delta_t").as_double();
            RCLCPP_INFO(this->get_logger(), "speed: %f", speed);
            RCLCPP_INFO(this->get_logger(), "delta_t: %f", delta_t);

            tf_listener = std::make_shared<tf2_ros::TransformListener>(tf_buffer);

            sub_pose = this->create_subscription<nav_msgs::msg::Odometry>(
                "/waffle2/odom_fhj", 10, std::bind(&DrivingNode::my_callback, this, std::placeholders::_1));

            sub_goal_pose = this->create_subscription<geometry_msgs::msg::PoseStamped>(
                "/goal_pose", 10, std::bind(&DrivingNode::goal_pose_callback, this, std::placeholders::_1));

            sub_map = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
                "/map", 10, std::bind(&DrivingNode::map_callback, this, std::placeholders::_1));


            pub_vel = this->create_publisher<geometry_msgs::msg::Twist>("/waffle2/cmd_vel", 10); // topic + QoS

            marker_pub = this->create_publisher<visualization_msgs::msg::Marker>("/goal_marker", 10);
        }

    private:
        // declare any subscriber / publisher / timer
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_pose;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_goal_pose;
        rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_map;

        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_vel;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub;

        nav_msgs::msg::Odometry odom_msg;
        geometry_msgs::msg::PoseStamped goal_msg;
        geometry_msgs::msg::Twist cmd_vel_msg;
        nav_msgs::msg::OccupancyGrid map_msg;

        tf2_ros::Buffer tf_buffer;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener;

        float speed = 0.0;
        float delta_t = 0.0;
        float x_current = 0.0;
        float y_current = 0.0;
        float theta_current = 0.0;
        float x_goal = 0.0;
        float y_goal = 0.0;
        float x_diff = 0.0;
        float y_diff = 0.0;
        float distance_new = 0.0;
        float distance_old = 1000.0;
        float x_pred = 0.0;
        float y_pred = 0.0;
        float omega_to_send = 0.0;
        float omega = 0.0;

        void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
        {
            map_msg = *msg; // Store the received map data
            // Handle the map data if needed
            RCLCPP_INFO(this->get_logger(), "Received map data");
        }

        void goal_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
        {
            goal_msg = *msg; // Store the received goal pose

            // Transform the goal pose from its current frame to the "odom" frame
            geometry_msgs::msg::TransformStamped transform_stamped;
            transform_stamped = tf_buffer.lookupTransform("waffle2/odom_frame_fhj", goal_msg.header.frame_id, tf2::TimePointZero);
            geometry_msgs::msg::PoseStamped transformed_goal;
            tf2::doTransform(goal_msg, transformed_goal, transform_stamped);
            goal_msg = transformed_goal; // Update the goal_msg with the transformed pose

            // Add a marker to visualize the goal
            visualization_msgs::msg::Marker goal_marker;
            goal_marker.header.frame_id = "waffle2/odom_frame_fhj";
            goal_marker.header.stamp = this->now();
            goal_marker.ns = "goal_marker";
            goal_marker.id = 0;
            goal_marker.type = visualization_msgs::msg::Marker::SPHERE;
            goal_marker.action = visualization_msgs::msg::Marker::ADD;
            goal_marker.pose = goal_msg.pose;
            goal_marker.scale.x = 0.2;
            goal_marker.scale.y = 0.2;
            goal_marker.scale.z = 0.2;
            goal_marker.color.a = 1.0;
            goal_marker.color.r = 1.0;
            goal_marker.color.g = 0.0;
            goal_marker.color.b = 0.0;
            marker_pub->publish(goal_marker);
        }

        void my_callback(const nav_msgs::msg::Odometry::SharedPtr odom_msg)
        {

            x_current = odom_msg->pose.pose.position.x;
            y_current = odom_msg->pose.pose.position.y;
            double qx = odom_msg->pose.pose.orientation.x;
            double qy = odom_msg->pose.pose.orientation.y;
            double qz = odom_msg->pose.pose.orientation.z;
            double qw = odom_msg->pose.pose.orientation.w;
            double siny_cosp = 2.0 * (qw * qz + qx * qy);
            double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
            theta_current = std::atan2(siny_cosp, cosy_cosp);

            x_goal = goal_msg.pose.position.x;
            y_goal = goal_msg.pose.position.y;

            distance_old = 1000.0;

            RCLCPP_WARN(this->get_logger(), "UPDATED GOAL goal pose: x=%f, y=%f",
                        goal_msg.pose.position.x, goal_msg.pose.position.y);

            for (float omega = -3.14; omega < 3.14; omega += 0.05)
            {
                x_pred = x_current + speed * (delta_t * cos(theta_current + omega / 2 * delta_t));
                y_pred = y_current + speed * (delta_t * sin(theta_current + omega / 2 * delta_t));

                // Convert predicted position to map coordinates
                int map_x = static_cast<int>((x_pred - map_msg.info.origin.position.x) / map_msg.info.resolution);
                int map_y = static_cast<int>((y_pred - map_msg.info.origin.position.y) / map_msg.info.resolution);

                // Check if the predicted position is within map bounds
                if (map_x >= 0 && map_x < static_cast<int>(map_msg.info.width) &&
                    map_y >= 0 && map_y < static_cast<int>(map_msg.info.height))
                {
                    // Get the index of the predicted position in the occupancy grid
                    int index = map_y * map_msg.info.width + map_x;

                    // Check if the cell is free (value 0) or occupied (value > 0)
                    if (map_msg.data[index] > 0)
                    {
                        continue; // Skip this omega if the position is occupied
                    }
                }
                else
                {
                    continue; // Skip this omega if the position is out of bounds
                }

                x_diff = x_goal - x_pred;
                y_diff = y_goal - y_pred;
                distance_new = sqrt(x_diff * x_diff + y_diff * y_diff);

                if (distance_new < distance_old)
                {
                    distance_old = distance_new;
                    omega_to_send = omega;
                }
            }

            // auto pose = call_pose_service("turtle2");
            // RCLCPP_INFO(this->get_logger(), "turtle2 pose: x=%f, y=%f", pose.position.x, pose.position.y);

            cmd_vel_msg.linear.x = speed;
            cmd_vel_msg.angular.z = omega_to_send;

            RCLCPP_INFO(this->get_logger(), "Publishing driving commands: x_vel=%f, omega_vel=%f",
                        cmd_vel_msg.linear.x, cmd_vel_msg.angular.z);

            pub_vel->publish(cmd_vel_msg);
        }
    };

}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    rclcpp::spin(std::make_shared<navv::DrivingNode>(options));
    rclcpp::shutdown();
    return 0;
}
