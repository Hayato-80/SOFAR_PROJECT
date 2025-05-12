#include "rclcpp/rclcpp.hpp"
// #include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <cmath>
#include <vector>

class create_map : public rclcpp::Node
{
public:
create_map() : Node("create_map")
  {
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local();

    this->declare_parameter<bool>("use_sim_time", false);
    bool use_sim_time = this->get_parameter("use_sim_time").as_bool();
    if (use_sim_time) {
        this->set_parameter(rclcpp::Parameter("use_sim_time", true));
    }

    // subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    //   "/scan", 10,
    //   std::bind(&lidar::lidar_callback, this, std::placeholders::_1));

    // map_subscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    //   "/map", 10,
    //   std::bind(&lidar::map_callback, this, std::placeholders::_1));

    publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
      "/map",qos);
    // publish_timer = create_wall_timer(100ms,    // rate
    //   [&](){callback_time();});
    publish_timer = create_wall_timer(
      std::chrono::milliseconds(100), // Publish every 100ms
      std::bind(&create_map::add_obstacles_and_publish_map, this));

    // Initialize the map
    initialize_map();

    add_obstacles_and_publish_map();

    broadcast_map_frame();
    
  }

private:
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher_;
  //rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscription_;
  rclcpp::TimerBase::SharedPtr publish_timer;

  nav_msgs::msg::OccupancyGrid map_;

  void initialize_map()
  {
    map_.info.resolution = 0.1; // 10 cm per cell
    map_.info.width = 100;      // 10 meters wide
    map_.info.height = 100;     // 10 meters tall
    map_.info.origin.position.x = 0.0;
    map_.info.origin.position.y = 0.0;
    map_.info.origin.position.z = 0.0;
    map_.info.origin.orientation.w = 1.0;

    // Initialize all cells as free (0)
    map_.data.resize(map_.info.width * map_.info.height, 0);
  }

  void draw_rect(int x_start, int y_start, int width, int height, int value){
    for(int y = y_start; y<y_start+height;y++){
      for(int x = x_start; x<x_start+width;x++){
        if(x>=0 && x < static_cast<int>(map_.info.width) &&
        y >= 0 && y < static_cast<int>(map_.info.height)){
          int index = y * map_.info.width + x;
          map_.data[index] = value; // Mark as occupied
        }
      }
    }
  }

  void add_obstacles_and_publish_map()
  {
    // Add obstacles at known positions
    draw_rect(20, 30, 10, 5, 100);
    draw_rect(50, 40, 10, 5, 100);
    draw_rect(10, 10, 10, 5, 100);

    // Publish the map
    map_.header.stamp = this->now();
    map_.header.frame_id = "map"; // Set the frame ID
    publisher_->publish(map_);
    RCLCPP_INFO(this->get_logger(), "Map with obstacles published");
  }
  
  void broadcast_map_frame()
  {
    static tf2_ros::StaticTransformBroadcaster static_broadcaster(this);

    geometry_msgs::msg::TransformStamped transform_stamped;

    transform_stamped.header.stamp = this->now();
    transform_stamped.header.frame_id = "world"; // Parent frame
    transform_stamped.child_frame_id = "map";    // Child frame

    transform_stamped.transform.translation.x = 0.0;
    transform_stamped.transform.translation.y = 0.0;
    transform_stamped.transform.translation.z = 0.0;

    transform_stamped.transform.rotation.x = 0.0;
    transform_stamped.transform.rotation.y = 0.0;
    transform_stamped.transform.rotation.z = 0.0;
    transform_stamped.transform.rotation.w = 1.0;

    static_broadcaster.sendTransform(transform_stamped);
    RCLCPP_INFO(this->get_logger(), "Broadcasting map frame");
  }

};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<create_map>());
    rclcpp::shutdown();
    return 0;
}
