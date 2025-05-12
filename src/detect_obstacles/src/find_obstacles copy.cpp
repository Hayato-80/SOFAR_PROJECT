#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>
#include <vector>
#include <limits>

class ObstacleDetector : public rclcpp::Node {
public:
    ObstacleDetector() : Node("obstacle_detector"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_) {
        // Subscribe to the /scan topic
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&ObstacleDetector::lidarCallback, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "ObstacleDetector node initialized.");

        obstacle_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("detected_obstacle", 10);
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    void lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
        // Thresholds for detecting a box-like obstacle
        const float min_box_size = 0.2;  // Minimum size of the box (meters)
        const float max_box_size = 1.0; // Maximum size of the box (meters)
        const float cluster_tolerance = 0.1; // Distance between points to consider them part of the same cluster

        std::vector<std::pair<float, float>> cluster_points; // Store points of a potential obstacle
        std::vector<std::pair<float, float>> detected_obstacles; // Store centroids of detected obstacles

        // Iterate through the LiDAR scan data
        for (size_t i = 0; i < scan->ranges.size(); ++i) {
            float distance = scan->ranges[i];
            if (distance > scan->range_min && distance < scan->range_max) {
                // Calculate the point's coordinates in the laser frame
                float angle = scan->angle_min + i * scan->angle_increment;
                float x = distance * std::cos(angle);
                float y = distance * std::sin(angle);

                // Check if the point belongs to the current cluster
                if (!cluster_points.empty()) {
                    float last_x = cluster_points.back().first;
                    float last_y = cluster_points.back().second;
                    float distance_to_last = std::hypot(x - last_x, y - last_y);

                    if (distance_to_last > cluster_tolerance) {
                        // End of the current cluster, process it
                        processCluster(cluster_points, min_box_size, max_box_size, detected_obstacles);
                        cluster_points.clear();
                    }
                }

                // Add the point to the current cluster
                cluster_points.emplace_back(x, y);
            }
        }

        // Process the last cluster
        if (!cluster_points.empty()) {
            processCluster(cluster_points, min_box_size, max_box_size, detected_obstacles);
        }

        // Transform and log detected obstacles
        for (const auto& obstacle : detected_obstacles) {
            geometry_msgs::msg::PointStamped obstacle_point;
            obstacle_point.header.frame_id = scan->header.frame_id; // Typically "base_scan" or "laser"
            obstacle_point.header.stamp = this->get_clock()->now();
            obstacle_point.point.x = obstacle.first;
            obstacle_point.point.y = obstacle.second;
            obstacle_point.point.z = 0.0;

            try {
                // Transform the point to the map frame
                geometry_msgs::msg::PointStamped transformed_point;
                transformed_point = tf_buffer_.transform(obstacle_point, "map", tf2::durationFromSec(1.0));

                RCLCPP_INFO(this->get_logger(), "Obstacle detected in map frame: x=%.2f, y=%.2f",
                            transformed_point.point.x, transformed_point.point.y);
            } catch (const tf2::TransformException& ex) {
                RCLCPP_WARN(this->get_logger(), "Failed to transform point to map frame: %s", ex.what());
            }
        }
    }

    void processCluster(const std::vector<std::pair<float, float>>& cluster_points,
                        float min_box_size, float max_box_size,
                        std::vector<std::pair<float, float>>& detected_obstacles) {
        // Calculate the bounding box size of the cluster
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

        // Check if the cluster matches the size of a box
        if (box_width >= min_box_size && box_width <= max_box_size &&
            box_height >= min_box_size && box_height <= max_box_size) {
            // Calculate the centroid of the cluster
            float centroid_x = (min_x + max_x) / 2.0;
            float centroid_y = (min_y + max_y) / 2.0;
            detected_obstacles.emplace_back(centroid_x, centroid_y);
        }
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ObstacleDetector>());
    rclcpp::shutdown();
    return 0;
}