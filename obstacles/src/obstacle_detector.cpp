#include <memory>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose.hpp"


class ObstacleDetectorNode : public rclcpp::Node
{
public:
    ObstacleDetectorNode() : Node("obstacle_detector")
    {
        costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "costmap",
            10,
            std::bind(&ObstacleDetectorNode::costmap_callback, this, std::placeholders::_1)
        );

        obstacle_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>(
            "/obstacles/poses",
            10
        );

        RCLCPP_INFO(this->get_logger(), "Costmap Obstacle Detector Node initialized.");
    }

private:

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr obstacle_pub_;

    void costmap_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        unsigned int width = msg->info.width;
        unsigned int height = msg->info.height;
        const std::vector<int8_t>& grid_data = msg->data;

        double resolution = msg->info.resolution;

        std::vector<geometry_msgs::msg::Pose> obstacle_list;

        for (size_t index = 0; index < grid_data.size(); ++index) {
            
            if (grid_data[index] == 100) { // this can be reduced to account for safe zones around the obstacles
                
                // Row-major layout inverse math: Convert 1D index to 2D grid metrics
                unsigned int cell_x = index % width;
                unsigned int cell_y = index / width;
                // double centroid_x = cell_x * resolution + resolution/2;
                // double centroid_y = cell_y * resolution + resolution/2;
                double centroid_x = cell_x * resolution;
                double centroid_y = cell_y * resolution;
                // in robot frame
                double x_m = centroid_x - width * resolution/2;
                double y_m = centroid_y - height * resolution/2;

                auto obstacle = geometry_msgs::msg::Pose();
                obstacle.position.x = x_m;
                obstacle.position.y = y_m;
                obstacle_list.push_back(obstacle);

                // centroid_x = cell_x * resolution + 3*resolution/4;
                // centroid_y = cell_y * resolution + 3*resolution/4;
                // // in robot frame
                // x_m = centroid_x - width * resolution/2;
                // y_m = centroid_y - height * resolution/2;

                // obstacle.position.x = x_m;
                // obstacle.position.y = y_m;
                // obstacle_list.push_back(obstacle);

                RCLCPP_INFO(this->get_logger(), "Obstacle at x = %f, y = %f", x_m, y_m);
            }
        }

        auto obstacles = geometry_msgs::msg::PoseArray();
        obstacles.poses = obstacle_list;
        obstacle_pub_->publish(obstacles);

        RCLCPP_INFO(this->get_logger(), "Published %lu obstacles", obstacle_list.size());

    }

    
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ObstacleDetectorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}