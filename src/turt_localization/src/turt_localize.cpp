#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <math.h>

#include <sensor_msgs/msg/joint_state.hpp> // receives joint states -- used for math

#include <tf2/LinearMath/Quaternion.h> // used in my transform I publish
#include <tf2_ros/transform_broadcaster.h> // how I send my transform

#include <nav_msgs/msg/odometry.hpp> // odometry I am publishing
#include <rclcpp/rclcpp.hpp> // generally used
#include "std_msgs/msg/string.hpp" // perhaps useful... probably not

using namespace std::chrono_literals;

class MyNode : public rclcpp::Node
{
  public:
    MyNode() : Node("my_node"), 
      x_(0.0), y_(0.0), theta_(0.0), 
      past_left_value(0.0), past_right_value(0.0),
      radius(0.033), width(0.287), first_reading(true) // put x, y, theta, last left/right pos, 
    {

        // init subscribers
        my_subscription = this->create_subscription<sensor_msgs::msg::JointState>(
                    "/joint_states", 10, std::bind(&MyNode::my_callback, this, std::placeholders::_1));

        // init publishers
        my_publisher = this->create_publisher<nav_msgs::msg::Odometry>("/odom_fhj", 10);   // topic + QoS
        
        tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this); // broadcasts the transformation

        past_time = this->now(); // used for velocity

        RCLCPP_INFO(this->get_logger(), "/odom_fhj node started");
    }

  private:
    // declare any subscriber / publisher / timer -- not say what they are, just declare them
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr my_subscription; 
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr my_publisher;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;

    // constant value of radius for calculation
    const double radius, width; 

    // declare past value variables
    double past_left_value, past_right_value;
    double x_, y_, theta_;
    bool first_reading;
    rclcpp::Time past_time;
    
    void my_callback(sensor_msgs::msg::JointState msg)
    {
      geometry_msgs::msg::TransformStamped transform;

      double leftPosition{msg.position[0]}, leftVelocity{msg.velocity[0]}, 
             rightPosition{msg.position[1]}, rightVelocity{msg.velocity[1]};
      
      // calculate values of x, y, theta: 
      if(first_reading == true)
      {
        past_left_value = leftPosition;
        past_right_value = rightPosition;
        first_reading = false;
      }
      //rclcpp::Time current_time = this->now();
      rclcpp::Time current_time = msg.header.stamp.sec != 0 || msg.header.stamp.nanosec != 0
                                  ? rclcpp::Time(msg.header.stamp)
                                  : this->now();
      double delta_t{(current_time - past_time).seconds()}; 

      double delta_q_l{leftPosition - past_left_value}, 
             delta_q_r{rightPosition - past_right_value};
      double deltaD{(radius * delta_q_l + radius * delta_q_r)/2.0}, 
             delta_theta{(radius * delta_q_l - radius * delta_q_r) / width};
      double calc_theta{theta_ + delta_theta};
      double calc_x{x_ + deltaD * cos((calc_theta + theta_)/2)}, 
             calc_y{y_ + deltaD * sin((calc_theta + theta_)/2)};

      // calculate vales of linear and angular velocity:
      double vel_x{(calc_x - x_)/delta_t}, 
             omega{(delta_theta)/delta_t};


      tf2::Quaternion quat;
    
      quat.setRPY(0, 0, calc_theta); // use calculated theta to determine quaternion

      nav_msgs::msg::Odometry out_msg; //

      // Header stuff
      out_msg.header.frame_id = "odom_frame_fhj"; // link frames
      out_msg.child_frame_id = "base_footprint"; // to the base frame
      out_msg.header.stamp = current_time;
      

      // Set position:
      out_msg.pose.pose.position.x = calc_x;
      out_msg.pose.pose.position.y = calc_y;
      out_msg.pose.pose.position.z = 0; // in 2d there is no z
      out_msg.pose.pose.orientation.x = quat.x();
      out_msg.pose.pose.orientation.y = quat.y();
      out_msg.pose.pose.orientation.z = quat.z();
      out_msg.pose.pose.orientation.w = quat.w();

      // Set twist:
      out_msg.twist.twist.linear.x = vel_x;
      out_msg.twist.twist.linear.y = 0;
      out_msg.twist.twist.linear.z = 0;
      out_msg.twist.twist.angular.x = 0;
      out_msg.twist.twist.angular.y = 0;
      out_msg.twist.twist.angular.z = omega;

      // Create transform:
      transform.transform.translation.x = calc_x;
      transform.transform.translation.y = calc_y;
      transform.transform.translation.z = 0;
      transform.transform.rotation.x = quat.x();
      transform.transform.rotation.y = quat.y();
      transform.transform.rotation.z = quat.z();
      transform.transform.rotation.w = quat.w();

      // transform.header.stamp = this->get_clock()->now();
      transform.header.stamp = current_time;
      transform.header.frame_id = "odom_frame_fhj";
      transform.child_frame_id = "base_footprint";

      my_publisher->publish(out_msg);
      tf_broadcaster->sendTransform(transform);


      // update past variables
      past_left_value = leftPosition;
      past_right_value = rightPosition;
      past_time = current_time;

      x_ = calc_x;
      y_ = calc_y;
      theta_ = calc_theta;
    }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MyNode>());
  rclcpp::shutdown();
  return 0;
}
