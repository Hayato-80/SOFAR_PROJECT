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
            default_speed = this->get_parameter("speed").as_double();
            delta_t = this->get_parameter("delta_t").as_double();
            RCLCPP_INFO(this->get_logger(), "speed: %f", speed);
            RCLCPP_INFO(this->get_logger(), "delta_t: %f", delta_t);

            tf_listener = std::make_shared<tf2_ros::TransformListener>(tf_buffer);

            sub_pose = this->create_subscription<nav_msgs::msg::Odometry>(
                "/odom_fhj", 10, std::bind(&DrivingNode::my_callback, this, std::placeholders::_1));

            sub_goal_pose = this->create_subscription<geometry_msgs::msg::PoseStamped>(
                "/goal_pose", 10, std::bind(&DrivingNode::goal_pose_callback, this, std::placeholders::_1));

            sub_map = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
                "/map", 10, std::bind(&DrivingNode::map_callback, this, std::placeholders::_1));

            pub_vel = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10); // topic + QoS

            marker_pub = this->create_publisher<visualization_msgs::msg::Marker>("/goal_marker", 10);

            marker_target_pub = this->create_publisher<visualization_msgs::msg::Marker>("/target_marker", 10);
        }

    private:
        // declare any subscriber / publisher / timer
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_pose;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_goal_pose;
        rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_map;

        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_vel;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_target_pub;

        nav_msgs::msg::Odometry odom_msg;
        geometry_msgs::msg::PoseStamped goal_msg;
        geometry_msgs::msg::Twist cmd_vel_msg;
        nav_msgs::msg::OccupancyGrid map_msg;

        tf2_ros::Buffer tf_buffer;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener;

        float speed = 0.0;
        float default_speed = 0.0;
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
        float x_pred_save = 0.0;
        float y_pred_save = 0.0;
        bool goal_received = false;

        void
        map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
        {
            map_msg = *msg; // Store the received map data
            // Handle the map data if needed
            RCLCPP_INFO(this->get_logger(), "Received map data");
        }

        void goal_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
        {
            goal_msg = *msg;      // Store the received goal pose
            goal_received = true; // Set the flag to indicate that the goal has been received

            speed = default_speed; // Reset speed to default

            // Transform the goal pose from its current frame to the "odom" frame
            geometry_msgs::msg::TransformStamped transform_stamped;
            transform_stamped = tf_buffer.lookupTransform("odom_frame_fhj", goal_msg.header.frame_id, tf2::TimePointZero);
            geometry_msgs::msg::PoseStamped transformed_goal;
            tf2::doTransform(goal_msg, transformed_goal, transform_stamped);
            goal_msg = transformed_goal; // Update the goal_msg with the transformed pose

            // Add a marker to visualize the goal
            visualization_msgs::msg::Marker goal_marker;
            goal_marker.header.frame_id = "odom_frame_fhj";
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
            marker_target_pub->publish(goal_marker);
        }

        void my_callback(const nav_msgs::msg::Odometry::SharedPtr odom_msg)
        {

            if (!goal_received)
            {
                RCLCPP_WARN(this->get_logger(), "Waiting for the first goal pose...");
                return; // Do nothing until the first goal pose is received
            }

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

            x_pred_save = 0.0;
            y_pred_save = 0.0;

            distance_old = 1000.0;

            RCLCPP_INFO(this->get_logger(), "UPDATED GOAL goal pose in odom: x=%f, y=%f",
                        goal_msg.pose.position.x, goal_msg.pose.position.y);

            for (float omega = -2.0; omega < 2.0; omega += 0.05)
            {
                x_pred = x_current + speed * (delta_t * cos(theta_current + omega / 2 * delta_t));
                y_pred = y_current + speed * (delta_t * sin(theta_current + omega / 2 * delta_t));

                bool is_free = true;
                for (float t = 0.0; t <= delta_t; t += 0.1)
                {
                    // Predict intermediate positions
                    float x_intermediate = x_current + speed * (t * cos(theta_current + omega / 2 * t));
                    float y_intermediate = y_current + speed * (t * sin(theta_current + omega / 2 * t));

                    // Convert intermediate position to map coordinates
                    int map_x = static_cast<int>((x_intermediate - map_msg.info.origin.position.x) / map_msg.info.resolution);
                    int map_y = static_cast<int>((y_intermediate - map_msg.info.origin.position.y) / map_msg.info.resolution);

                    int index = map_y * map_msg.info.width + map_x;

                    // Check if the cell is free (value 0) or occupied (value > 0)
                    if (index < 0 || index >= static_cast<int>(map_msg.data.size()) || map_msg.data[index] > 0)
                    {
                        is_free = false; // Mark as not free if any position is occupied
                        break;
                    }
                }

                if (!is_free)
                {
                    continue; // Skip this omega if any intermediate position is occupied
                }

                x_diff = x_goal - x_pred;
                y_diff = y_goal - y_pred;
                distance_new = sqrt(x_diff * x_diff + y_diff * y_diff);

                if (distance_new < distance_old)
                {
                    distance_old = distance_new;
                    omega_to_send = omega;
                    x_pred_save = x_pred;
                    y_pred_save = y_pred;
                }
            }

            // Check if the predicted position is close to the goal

            float distance_to_goal = sqrt(pow(x_goal - x_current, 2) + pow(y_goal - y_current, 2));
            RCLCPP_INFO(this->get_logger(), "Distance to goal: %f", distance_to_goal);

            float goal_threshold = 0.7; 
            float stop_threshold = 0.05;
            if (distance_to_goal < goal_threshold && distance_to_goal > stop_threshold)
            {
                speed = std::max(0.1f, default_speed * (distance_to_goal / goal_threshold)); // Smoothly slow down as it approaches the goal
            }
            else if (distance_to_goal < stop_threshold)
            {
                speed = 0.0;
                omega_to_send = 0.0; // Stop if very close to the goal
                RCLCPP_INFO(this->get_logger(), "Goal reached!");
                goal_received = false; // Reset the goal received flag
            }

            // Add a marker to visualize the goal
            visualization_msgs::msg::Marker target_marker;
            target_marker.header.frame_id = "odom_frame_fhj";
            target_marker.header.stamp = this->now();
            target_marker.ns = "target_marker";
            target_marker.id = 0;
            target_marker.type = visualization_msgs::msg::Marker::SPHERE;
            target_marker.action = visualization_msgs::msg::Marker::ADD;
            target_marker.pose.position.x = x_pred_save;
            target_marker.pose.position.y = y_pred_save;
            target_marker.pose.position.z = 0.0;    // Assuming a 2D plane
            target_marker.pose.orientation.w = 1.0; // Neutral orientation
            target_marker.scale.x = 0.15;
            target_marker.scale.y = 0.15;
            target_marker.scale.z = 0.15;
            target_marker.color.a = 1.0;
            target_marker.color.r = 0.0;
            target_marker.color.g = 1.0; // Green color
            target_marker.color.b = 0.0;
            marker_pub->publish(target_marker);

            // auto pose = call_pose_service("turtle2");
            // RCLCPP_INFO(this->get_logger(), "turtle2 pose: x=%f, y=%f", pose.position.x, pose.position.y);

            cmd_vel_msg.linear.x = speed;
            cmd_vel_msg.angular.z = -omega_to_send;

            RCLCPP_INFO(this->get_logger(), "Current position in odom: x=%f, y=%f, theta=%f",
                        x_current, y_current, theta_current);

            RCLCPP_INFO(this->get_logger(), "Publishing driving commands:                  x_vel=%f, omega_vel=%f",
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
