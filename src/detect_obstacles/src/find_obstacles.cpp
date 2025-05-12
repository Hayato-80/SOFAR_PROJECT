#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>
#include <vector>
#include <limits>
#include <tf2_ros/transform_broadcaster.h>

class ObstacleDetector : public rclcpp::Node {
public:
    ObstacleDetector() : Node("obstacle_detector"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_) {
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&ObstacleDetector::lidarCallback, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "ObstacleDetector node initialized.");

        grid_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 1);

        // Initialize occupancy grid
        
        grid_.header.frame_id = "map";
        grid_.info.resolution = 0.1; // 10cm per cell
        grid_.info.width = 200;      // 20m x 20m grid
        grid_.info.height = 200;
        grid_.info.origin.position.x = -10.0; // Centered at (0,0)
        grid_.info.origin.position.y = -10.0;
        grid_.info.origin.position.z = 0.0;
        grid_.info.origin.orientation.w = 1.0;
        grid_.data.assign(grid_.info.width * grid_.info.height, 0);
        
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_pub_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    nav_msgs::msg::OccupancyGrid grid_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    struct ObstacleRect {
            float min_x, max_x, min_y, max_y;
    };

    void lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
        // geometry_msgs::msg::TransformStamped map_to_odom;
        // map_to_odom.header.stamp = scan->header.stamp;
        // map_to_odom.header.frame_id = "map";
        // map_to_odom.child_frame_id = "odom_frame_fhj";
        // map_to_odom.transform.translation.x = 0.0;
        // map_to_odom.transform.translation.y = 0.0;
        // map_to_odom.transform.translation.z = 0.0;
        // map_to_odom.transform.rotation.x = 0.0;
        // map_to_odom.transform.rotation.y = 0.0;
        // map_to_odom.transform.rotation.z = 0.0;
        // map_to_odom.transform.rotation.w = 1.0;
        //tf_broadcaster_->sendTransform(map_to_odom);
        
        // cluster settings
        const float min_box_size = 0.05;
        const float max_box_size = 2.0;
        const float cluster_tolerance = 0.5;

        std::vector<std::pair<float, float>> cluster_points;
        
        // std::vector<std::pair<float, float>> detected_obstacles;
        std::vector<ObstacleRect> detected_obstacles;

        // Iterate through the LiDAR scan data
        for (size_t i = 0; i < scan->ranges.size(); ++i) {
            float distance = scan->ranges[i];
            if (distance > scan->range_min && distance < scan->range_max) {
                float angle = scan->angle_min + i * scan->angle_increment;
                float x = distance * std::cos(angle);
                float y = distance * std::sin(angle);

                if (!cluster_points.empty()) {
                    float last_x = cluster_points.back().first;
                    float last_y = cluster_points.back().second;
                    float distance_to_last = std::hypot(x - last_x, y - last_y);

                    if (distance_to_last > cluster_tolerance) {
                        processCluster(cluster_points, min_box_size, max_box_size, detected_obstacles);
                        cluster_points.clear();
                    }
                }
                cluster_points.emplace_back(x, y);
            }
        }
        if (!cluster_points.empty()) {
            processCluster(cluster_points, min_box_size, max_box_size, detected_obstacles);
        }

        // // Clear grid
        // std::fill(grid_.data.begin(), grid_.data.end(), 0);

        // // Transform and mark detected obstacles in the grid
        // for (const auto& obstacle : detected_obstacles) {
        //     geometry_msgs::msg::PointStamped obstacle_point;
        //     obstacle_point.header.frame_id = scan->header.frame_id;
        //     obstacle_point.header.stamp = this->get_clock()->now();
        //     obstacle_point.point.x = obstacle.first;
        //     obstacle_point.point.y = obstacle.second;
        //     obstacle_point.point.z = 0.0;

        //     try {
        //         geometry_msgs::msg::PointStamped transformed_point;
        //         transformed_point = tf_buffer_.transform(obstacle_point, "map", tf2::durationFromSec(1.0));

        //         // Convert map coordinates to grid indices
        //         int mx = static_cast<int>((transformed_point.point.x - grid_.info.origin.position.x) / grid_.info.resolution);
        //         int my = static_cast<int>((transformed_point.point.y - grid_.info.origin.position.y) / grid_.info.resolution);

        //         if (mx >= 0 && mx < static_cast<int>(grid_.info.width) &&
        //             my >= 0 && my < static_cast<int>(grid_.info.height)) {
        //             grid_.data[my * grid_.info.width + mx] = 100; // Mark as occupied
        //         }
        //     } catch (const tf2::TransformException& ex) {
        //         RCLCPP_WARN(this->get_logger(), "Failed to transform point to map frame: %s", ex.what());
        //     }
        // }

        processCluster(cluster_points, min_box_size, max_box_size, detected_obstacles);

        // Clear grid
        std::fill(grid_.data.begin(), grid_.data.end(), 0);

        // For each rectangle, mark all grid cells inside as occupied
        for (const auto& rect : detected_obstacles) {
            for (float x = rect.min_x; x <= rect.max_x; x += grid_.info.resolution) {
                for (float y = rect.min_y; y <= rect.max_y; y += grid_.info.resolution) {
                    geometry_msgs::msg::PointStamped obstacle_point;
                    obstacle_point.header.frame_id = scan->header.frame_id;
                    //obstacle_point.header.stamp = this->get_clock()->now();
                    obstacle_point.header.stamp = scan->header.stamp;
                    obstacle_point.point.x = x;
                    obstacle_point.point.y = y;
                    obstacle_point.point.z = 0.0;

                    try {
                        geometry_msgs::msg::PointStamped transformed_point;
                        transformed_point = tf_buffer_.transform(obstacle_point, "map", tf2::durationFromSec(1.0));
                        int mx = static_cast<int>((transformed_point.point.x - grid_.info.origin.position.x) / grid_.info.resolution);
                        int my = static_cast<int>((transformed_point.point.y - grid_.info.origin.position.y) / grid_.info.resolution);
                        if (mx >= 0 && mx < static_cast<int>(grid_.info.width) &&
                            my >= 0 && my < static_cast<int>(grid_.info.height)) {
                            grid_.data[my * grid_.info.width + mx] = 100;
                        }
                        for (int dx = -1; dx <= 1; ++dx) {
                            for (int dy = -1; dy <= 1; ++dy) {
                                int nx = mx + dx;
                                int ny = my + dy;
                                if (nx >= 0 && nx < static_cast<int>(grid_.info.width) &&
                                    ny >= 0 && ny < static_cast<int>(grid_.info.height)) {
                                    grid_.data[ny * grid_.info.width + nx] = 100;
                                }
                            }
                        }   
                    } catch (const tf2::TransformException& ex) {
                        RCLCPP_WARN(this->get_logger(), "Failed to transform point to map frame: %s", ex.what());
                    }
                }
            }
        }

        grid_.header.stamp = this->get_clock()->now();
        grid_pub_->publish(grid_);
    }

    void processCluster(const std::vector<std::pair<float, float>>& cluster_points,
                        float min_box_size, float max_box_size,
                        std::vector<ObstacleRect>& detected_rects) {
        float min_x = std::numeric_limits<float>::infinity();
        float max_x = -std::numeric_limits<float>::infinity();
        float min_y = std::numeric_limits<float>::infinity();
        float max_y = -std::numeric_limits<float>::infinity();

        for (const auto& point : cluster_points) {
            min_x = std::min(min_x, point.first);
            max_x = std::max(max_x, point.first);
            min_y = std::min(min_y, point.second);
            max_y = std::max(max_y, point.second);
        }

        float box_width = max_x - min_x;
        float box_height = max_y - min_y;

        if (box_width >= min_box_size && box_width <= max_box_size &&
            box_height >= min_box_size && box_height <= max_box_size) {
            detected_rects.push_back({min_x, max_x, min_y, max_y});
        }
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    rclcpp::spin(std::make_shared<ObstacleDetector>());
    rclcpp::shutdown();
    return 0;
}