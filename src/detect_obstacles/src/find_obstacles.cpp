#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include <cmath>
#include <vector>

class lidar : public rclcpp::Node
{
public:
lidar() : Node("range_to_origin")
  {
    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10,
      std::bind(&lidar::lidar_callback, this, std::placeholders::_1));

    map_subscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map", 10,
      std::bind(&lidar::map_callback, this, std::placeholders::_1));

    publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
      "/ocupancy_grid", 10);
    // publish_timer = create_wall_timer(100ms,    // rate
    //   [&](){callback_time();});
    
  }

private:
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscription_;
  // rclcpp::TimerBase::SharedPtr publish_timer;

  nav_msgs::msg::OccupancyGrid received_map_;
  bool map_received = false;

  void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    received_map_ = *msg;
    map_received = true;
    RCLCPP_INFO(this->get_logger(), "Map is received");
  }

  void lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    if(!map_received){
      RCLCPP_WARN(this->get_logger(), "No map is received yet");
      return;
    }

    nav_msgs::msg::OccupancyGrid updated_map = received_map_;

    // Calculate the angle of the current range
    std::vector<float> ranges = msg->ranges;
    for(size_t i=0; i<ranges.size(); i++){
      float angle = msg->angle_min + i * msg->angle_increment;
      float range = ranges[i];

      // Calculate the x and y coordinates of the point
      float x = range * cos(angle);
      float y = range * sin(angle);

      // Convert to map coordinates
      int map_x = static_cast<int>((x - updated_map.info.origin.position.x) / updated_map.info.resolution);
      int map_y = static_cast<int>((y - updated_map.info.origin.position.y) / updated_map.info.resolution);

      // Check if the coordinates are within the map bounds
      if(map_x >= 0 && map_x < updated_map.info.width && map_y >= 0 && map_y < updated_map.info.height){
        // Update the occupancy grid
        updated_map.data[map_y * updated_map.info.width + map_x] = 100; // Mark as occupied
      }
    }
    updated_map.header.stamp = this->now();
    publisher_->publish(updated_map);

  }

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<lidar>());
  rclcpp::shutdown();
  return 0;
}
