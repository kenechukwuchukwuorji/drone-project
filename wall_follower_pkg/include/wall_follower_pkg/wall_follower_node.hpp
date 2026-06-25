#pragma once

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include <Eigen/Dense>

#include <vector>

class WallFollowerNode : public rclcpp::Node
{
public:
    WallFollowerNode();

private:

    struct Segment
    {
        std::vector<Eigen::Vector2d> points;
        double length = 0.0;
    };

    struct Wall
    {
        Eigen::Vector2d centroid;
        Eigen::Vector2d direction;
        double length = 0.0;
        double rmse = 0.0;
    };

    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void poseCallback(const tf2_msgs::msg::TFMessage::SharedPtr msg);
    void delay_publish();

    bool detectWall(const sensor_msgs::msg::LaserScan& scan, Wall& wall);
    std::vector<Segment> segmentScan(const sensor_msgs::msg::LaserScan& scan);
    bool fitLine(const Segment& seg, Wall& wall);

    Eigen::Vector2d computeWaypointBody(const Wall& wall);
    geometry_msgs::msg::Point toWorld(const Eigen::Vector2d& p_body);

    bool has_pose_ = false;
    bool wall_exit_buffer_ = false;
    double wall_threshold_ = 2.0;

    Eigen::Vector2d last_wall_pos_;
    bool tracking_wall_exit_ = false;
    double exit_distance_ = 0.0;
    geometry_msgs::msg::Point last_wall_wp_;
    bool detected_wall_;

    Eigen::Vector2d curr_pos_;
    double curr_yaw_ = 0.0;

    int wall_counter_ = 0;

    Eigen::Vector2d goal_;
    bool has_goal_ = false;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr pose_sub_;

    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr wp_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr active_pub_;

    rclcpp::TimerBase::SharedPtr delay_timer_;
    
};