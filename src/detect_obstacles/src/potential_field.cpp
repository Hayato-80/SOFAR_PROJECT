#include <memory>
#include <vector>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2/LinearMath/Quaternion.h"

class PotentialField : public rclcpp::Node {
public:
    PotentialField() : Node("potential_field_node")
  {
    //this->declare_parameter("use_sim_time", true);
    //auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local();

    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10,
      std::bind(&PotentialField::lidar_callback, this, std::placeholders::_1));

    goal_pub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/goal_pose", 10, std::bind(&PotentialField::goal_callback, this , std::placeholders::_1));

    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    att_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("attraction_vector", 10);
    rep_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("repulsion_vector", 10);
    fin_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("final_vector", 10);

    timer_ = create_wall_timer(std::chrono::milliseconds(200), std::bind(&PotentialField::controller, this));
    
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    V_attraction_ = {0.0, 0.0};
    V_repulsion_ = {0.0, 0.0};
  }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr att_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr rep_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr fin_pub_;

    std::vector<float> V_attraction_;
    std::vector<float> V_repulsion_;
     

    rclcpp::TimerBase::SharedPtr timer_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    geometry_msgs::msg::PoseStamped goal_pose_;

    bool if_goal_ = false;

    geometry_msgs::msg::PoseStamped create_vector_pose(double x, double y) {
        geometry_msgs::msg::PoseStamped vector;
        vector.header.frame_id = "waffle2/base_footprint";
        vector.header.stamp = this->get_clock()->now();
        vector.pose.position.x = x;
        vector.pose.position.y = y;
        vector.pose.position.z = 0.0;
        double angle = atan2(y, x);
        tf2::Quaternion q;
        q.setRPY(0, 0, angle);
        vector.pose.orientation.x = q.x();
        vector.pose.orientation.y = q.y();
        vector.pose.orientation.z = q.z();
        vector.pose.orientation.w = q.w();
        return vector;
    }

    void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        goal_pose_ = *msg;
        RCLCPP_INFO(this->get_logger(), "Goal pose received");
        if_goal_ = true;
    }

    void controller()
    {
        geometry_msgs::msg::TransformStamped tf;
        geometry_msgs::msg::PoseStamped goal_in_odom;
        geometry_msgs::msg::Twist cmd_msg;

        if(!if_goal_){
            RCLCPP_INFO(this->get_logger(), "Waiting for goal pose");
            return;
        }

        
        // try{
        //     //tf = tf_buffer_ -> lookupTransform("map","base_link", tf2::TimePointZero);
        //     tf = tf_buffer_ -> lookupTransform("odom","base_link", rclcpp::Time(0));

        // } catch(tf2::TransformException &ex){
        //     RCLCPP_WARN(this->get_logger(), "Could not transform base_link to map: %s", ex.what());
        //     return;
        // }
        geometry_msgs::msg::PoseStamped goal_pose = goal_pose_;
        goal_pose.pose.orientation.x = 0.0;
        goal_pose.pose.orientation.y = 0.0;
        goal_pose.pose.orientation.z = 0.0;
        goal_pose.pose.orientation.w = 1.0;
        //goal_pose.header.stamp = this->get_clock()->now();
        
        try {
            goal_in_odom = tf_buffer_->transform(goal_pose, "base_footprint");
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "Transform to base_link failed: %s", ex.what());
            return;
        }
        geometry_msgs::msg::TransformStamped tf_odom_to_base_footprint;
        //geometry_msgs::msg::TransformStamped tf_base_footprint_to_base_link;
        try {
            // First, get the transform from odom to base_footprint
            tf_odom_to_base_footprint = tf_buffer_->lookupTransform("odom_frame_fhj", "base_footprint", tf2::TimePointZero);
            
            // Then, get the transform from base_footprint to base_link
            //tf_base_footprint_to_base_link = tf_buffer_->lookupTransform("base_footprint", "base_link", tf2::TimePointZero);
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "Could not transform from odom to base_footprint or base_footprint to base_link: %s", ex.what());
            return;
        }

        // Combine the transforms
        //geometry_msgs::msg::TransformStamped tf_combined;
        // tf_combined.header.stamp = tf_odom_to_base_footprint.header.stamp;
        // tf_combined.header.stamp = tf_odom_to_base_footprint.header.stamp;
        // tf_combined.transform.translation.x = tf_odom_to_base_footprint.transform.translation.x + tf_base_footprint_to_base_link.transform.translation.x;
        // tf_combined.transform.translation.y = tf_odom_to_base_footprint.transform.translation.y + tf_base_footprint_to_base_link.transform.translation.y;
        // tf_combined.transform.translation.z = tf_odom_to_base_footprint.transform.translation.z + tf_base_footprint_to_base_link.transform.translation.z;

        float dx = goal_in_odom.pose.position.x;
        float dy = goal_in_odom.pose.position.y;

        // float dx = goal_in_odom.pose.position.x - tf_odom_to_base_footprint.transform.translation.x;
        // float dy = goal_in_odom.pose.position.y - tf_odom_to_base_footprint.transform.translation.y;

        // float dx = goal_pose_.pose.position.x - tf.transform.translation.x;
        // float dy = goal_pose_.pose.position.y - tf.transform.translation.y;
        
        double distance = sqrt(dx*dx+dy*dy);

        if(distance < 0.1){
            RCLCPP_INFO(this->get_logger(), "Goal reached");
            geometry_msgs::msg::Twist stop_msg;
            stop_msg.linear.x = 0.0;
            stop_msg.angular.z = 0.0;
            cmd_pub_->publish(stop_msg);
            if_goal_ = false;
            return;
        }
        V_attraction_ = {dx, dy};

        double repulsion_scale = 0.01;  // try smaller values like 0.3–0.6
        double attraction_scale = 1.0;
        float x_final = attraction_scale*V_attraction_[0] + repulsion_scale*V_repulsion_[0];
        float y_final = attraction_scale*V_attraction_[1] + repulsion_scale*V_repulsion_[1];

        double target_angle = std::atan2(y_final, x_final);
        
        // tf2::Quaternion q(
        //     // tf.transform.rotation.x,
        //     // tf.transform.rotation.y,
        //     // tf.transform.rotation.z,
        //     // tf.transform.rotation.w
        //     tf_odom_to_base_footprint.transform.rotation.x,
        //     tf_odom_to_base_footprint.transform.rotation.y,
        //     tf_odom_to_base_footprint.transform.rotation.z,
        //     tf_odom_to_base_footprint.transform.rotation.w
        // );
        // tf2::Matrix3x3 m(q);
        // double roll, pitch, yaw;
        // m.getRPY(roll, pitch, yaw);
        
        //double angle_diff = target_angle - yaw;

        double angle_diff = target_angle;
        
        // Normalize angle_diff to [-π, π]
        while (angle_diff > M_PI) angle_diff -= 2 * M_PI;
        while (angle_diff < -M_PI) angle_diff += 2 * M_PI;

        

        double angle_tolerance = 0.05;  // ~3 degrees
        if (std::abs(angle_diff) > angle_tolerance) {
            geometry_msgs::msg::Twist cmd_msg;
            cmd_msg.linear.x = 0.05;  // Allow small forward motion while turning
            double angular_speed = angle_diff;
            if (angular_speed > 1.0) angular_speed = 1.0;
            if (angular_speed < -1.0) angular_speed = -1.0;
            cmd_msg.angular.z = angular_speed;
            cmd_pub_->publish(cmd_msg);
            return;
        }

        double linear_vel = std::min(distance, 0.2);

        // Using if-else for clamping the linear velocity
        if (linear_vel > 0.2) {
            linear_vel = 0.2;
        }

        cmd_msg.linear.x = linear_vel;
        cmd_msg.angular.z = 0.0;
        cmd_pub_->publish(cmd_msg);
        
        //RCLCPP_INFO(this->get_logger(), "Linear velocity: %f, Angular velocity: %f", linear_vel, angular_vel);
        
        auto attraction_vector = create_vector_pose(V_attraction_[0], V_attraction_[1]);
        auto repulsion_vector = create_vector_pose(V_repulsion_[0], V_repulsion_[1]);
        auto final_vector = create_vector_pose(x_final, y_final);
        att_pub_->publish(attraction_vector);
        rep_pub_->publish(repulsion_vector);
        fin_pub_->publish(final_vector);
    }

    void lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        msg->header.stamp = this->get_clock()->now();
        // Calculate the angle of the current range
        double angle_min = msg->angle_min;
        double angle_increment = msg->angle_increment;

        float x_range = 0.0;
        float y_range = 0.0;

        for(size_t i=0; i< msg->ranges.size(); i++){
            float ranges = msg->ranges[i];
            if(ranges < 0.6 && ranges >0.08){
                double angle = angle_min + i * angle_increment;
                x_range -= (1.0/ranges) * cos(angle);
                y_range -= (1.0/ranges) * sin(angle);
            }
            
        }
        V_repulsion_ = {x_range, y_range};

    }

};

int main(int argc, char * argv[])
{   
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PotentialField>());
    rclcpp::shutdown();
    return 0;

    // rclcpp::init(argc, argv);
    // auto node = std::make_shared<PotentialField>();
    // rclcpp::executors::MultiThreadedExecutor exec;
    // exec.add_node(node);
    // exec.spin();
    // rclcpp::shutdown();
    // return 0;
}
