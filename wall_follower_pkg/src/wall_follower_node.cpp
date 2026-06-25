#include "wall_follower_pkg/wall_follower_node.hpp"

#include <cmath>
#include <algorithm>

using std::placeholders::_1;
using namespace std::chrono_literals;

WallFollowerNode::WallFollowerNode()
: Node("wall_follower_node")
{
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10,
        std::bind(&WallFollowerNode::scanCallback, this, _1));

    pose_sub_ = create_subscription<tf2_msgs::msg::TFMessage>(
        "/tf", 10,
        std::bind(&WallFollowerNode::poseCallback, this, _1));

    wp_pub_ = create_publisher<geometry_msgs::msg::Point>(
        "/px4_0/waypoint", 10);

    active_pub_ = create_publisher<std_msgs::msg::Bool>(
        "/wall_follow_active", 10);

    delay_timer_ = create_wall_timer(5s, std::bind(&WallFollowerNode::delay_publish, this));
    delay_timer_->cancel();

    goal_ = Eigen::Vector2d(0.0, 10.0);
    has_goal_ = true;
}

/* ---------------- POSE ---------------- */

void WallFollowerNode::poseCallback(const tf2_msgs::msg::TFMessage::SharedPtr msg)
{
    if (msg->transforms.empty()) return;

    const auto& t = msg->transforms[0].transform;

    curr_pos_ = Eigen::Vector2d(
        t.translation.x,
        t.translation.y);

    double qx = t.rotation.x;
    double qy = t.rotation.y;
    double qz = t.rotation.z;
    double qw = t.rotation.w;

    curr_yaw_ = std::atan2(
        2.0 * (qw * qz + qx * qy),
        1.0 - 2.0 * (qy * qy + qz * qz));

    has_pose_ = true;
}

/* ---------------- MAIN CALLBACK ---------------- */

void WallFollowerNode::scanCallback(
    const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    if (!has_pose_ || !has_goal_) return;

    Wall wall;
    bool detected = detectWall(*msg, wall);
    std_msgs::msg::Bool active_msg;
    detected_wall_ = detected;

    geometry_msgs::msg::Point target;

    if (detected)
    {
        wall_counter_++;
        RCLCPP_INFO(this->get_logger(), "Wall detected");

        if (wall_counter_ >= 3)
        {
            Eigen::Vector2d wp_body = computeWaypointBody(wall);
            target = toWorld(wp_body);
            active_msg.data = true;

            active_pub_->publish(active_msg);
        }
        else
        {
            // unstable detection → go to goal
            // target.x = goal_.x();
            // target.y = goal_.y();
            // target.z = 5.0;
            // RCLCPP_INFO(this->get_logger(), "unstable direction, go to goal");
            active_msg.data = true;
            

            active_pub_->publish(active_msg);
            return;
        }
    }
    else
    {
        wall_counter_ = 0;

        // if (delay_timer_->is_canceled()) {
        //     delay_timer_->reset();
        // }

        std_msgs::msg::Bool active_msg;
        active_msg.data = false;
        active_pub_->publish(active_msg);
        
        
        // target.x = goal_.x();
        // target.y = goal_.y();
        // target.z = 5.0;
        RCLCPP_INFO(this->get_logger(), "no wall detected, returning to APF");
        return; 
    }
    wp_pub_->publish(target);
}

// void WallFollowerNode::scanCallback(
//     const sensor_msgs::msg::LaserScan::SharedPtr msg)
// {
//     if (!has_pose_ || !has_goal_) return;

//     Wall wall;
//     bool detected = detectWall(*msg, wall);
//     std_msgs::msg::Bool active_msg;

//     geometry_msgs::msg::Point target;

//     // -------------------------------------------------
//     // CASE 1: WALL DETECTED (NORMAL OPERATION)
//     // -------------------------------------------------
//     if (detected)
//     {
//         wall_counter_++;
//         tracking_wall_exit_ = false;
//         exit_distance_ = 0.0;

//         active_msg.data = true;
//         active_pub_->publish(active_msg);

//         if (wall_counter_ >= 3)
//         {
//             Eigen::Vector2d wp_body = computeWaypointBody(wall);
//             target = toWorld(wp_body);

//             last_wall_pos_ = curr_pos_;   // anchor exit tracking
//         }
//         else
//         {
//             return;
//         }

//         wp_pub_->publish(target);
//         return;
//     }

//     // -------------------------------------------------
//     // CASE 2: WALL JUST LOST → START EXIT TRACKING
//     // -------------------------------------------------
//     wall_counter_ = 0;

//     if (!tracking_wall_exit_)
//     {
//         tracking_wall_exit_ = true;
//         exit_distance_ = 0.0;
//         last_wall_pos_ = curr_pos_;
//         active_msg.data = true;
//         active_pub_->publish(active_msg);
//     }

//     // -------------------------------------------------
//     // CASE 3: CONTINUE LAST WALL TRAJECTORY
//     // -------------------------------------------------
//     if (tracking_wall_exit_)
//     {
//         active_msg.data = true;
//         active_pub_->publish(active_msg);

//         double d = (curr_pos_ - last_wall_pos_).norm();
//         exit_distance_ = d;

//         // IMPORTANT:
//         // reuse LAST computed wall-follow waypoint direction implicitly
//         // (we do NOT recompute anything)

//         target = last_wall_wp_;

//         wp_pub_->publish(target);

//         // EXIT AFTER 2m REAL MOTION
//         if (exit_distance_ >= 2.0)
//         {
//             tracking_wall_exit_ = false;
//         }

//         return;
//     }

//     // -------------------------------------------------
//     // CASE 4: NORMAL GOAL MODE
//     // -------------------------------------------------
//     active_msg.data = false;
//     active_pub_->publish(active_msg);
//     wp_pub_->publish(target);
// }

void WallFollowerNode::delay_publish(){
    delay_timer_->cancel();
    if(!detected_wall_){
        std_msgs::msg::Bool active_msg;
        active_msg.data = false;
        active_pub_->publish(active_msg);
    }
}

/* ---------------- WALL DETECTION ---------------- */

std::vector<WallFollowerNode::Segment>
WallFollowerNode::segmentScan(const sensor_msgs::msg::LaserScan& scan)
{
    std::vector<Segment> segments;

    Segment current;

    const double gap_threshold = 0.3;

    Eigen::Vector2d prev;

    bool first = true;

    for (size_t i = 0; i < scan.ranges.size(); i++)
    {
        double r = scan.ranges[i];
        if(r > wall_threshold_) continue;    // don't consider walls farther than the threshold
        if (!std::isfinite(r)) continue;

        double angle = scan.angle_min + i * scan.angle_increment;

        Eigen::Vector2d p(r * std::cos(angle), r * std::sin(angle));

        if (!first)
        {
            if ((p - prev).norm() > gap_threshold)
            {
                if (current.points.size() > 5)
                    segments.push_back(current);

                current = Segment();
            }
        }

        current.points.push_back(p);
        prev = p;
        first = false;
    }

    if (current.points.size() > 5)
        segments.push_back(current);

    return segments;
}

bool WallFollowerNode::fitLine(const Segment& seg, Wall& wall)
{
    Eigen::Vector2d mean(0,0);

    for (auto& p : seg.points)
        mean += p;

    mean /= seg.points.size();

    Eigen::Matrix2d cov = Eigen::Matrix2d::Zero();

    for (auto& p : seg.points)
    {
        Eigen::Vector2d d = p - mean;
        cov += d * d.transpose();
    }

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> solver(cov);

    Eigen::Vector2d dir = solver.eigenvectors().col(1).normalized();

    double rmse = 0.0;

    for (auto& p : seg.points)
    {
        Eigen::Vector2d d = p - mean;
        double dist = std::abs(d.x() * dir.y() - d.y() * dir.x());
        rmse += dist * dist;
    }

    rmse = std::sqrt(rmse / seg.points.size());

    wall.centroid = mean;
    wall.direction = dir;
    wall.length = seg.points.back().norm() - seg.points.front().norm();
    wall.rmse = rmse;

    return true;
}

bool WallFollowerNode::detectWall(
    const sensor_msgs::msg::LaserScan& scan,
    Wall& best_wall)
{
    auto segments = segmentScan(scan);

    double best_score = 0.0;
    bool found = false;

    for (auto& seg : segments)
    {
        Wall w;
        fitLine(seg, w);

        double length = seg.points.size() * 0.05;

        if (length < 2.5) continue;
        if (w.rmse > 0.08) continue;

        double score = length - w.rmse * 10.0;

        if (score > best_score)
        {
            best_score = score;
            best_wall = w;
            best_wall.length = length;
            found = true;
        }
    }

    return found;
}

/* ---------------- WAYPOINT ---------------- */

Eigen::Vector2d WallFollowerNode::computeWaypointBody(const Wall& wall)
{
    const double lookahead = 2.0;
    const double dist_from_wall = 2.0; //1.5;

    Eigen::Vector2d dir = wall.direction.normalized();

    // left normal (counter-clockwise)
    Eigen::Vector2d normal(-dir.y(), dir.x());

    Eigen::Vector2d target =
        wall.centroid +
        lookahead * dir +
        dist_from_wall * normal;

    return target;
}

geometry_msgs::msg::Point WallFollowerNode::toWorld(const Eigen::Vector2d& p)
{
    geometry_msgs::msg::Point out;

    double x = p.x();
    double y = p.y();

    out.x = curr_pos_.x()
          + std::cos(curr_yaw_) * x
          - std::sin(curr_yaw_) * y;

    out.y = curr_pos_.y()
          + std::sin(curr_yaw_) * x
          + std::cos(curr_yaw_) * y;

    out.z = 5.0;

    return out;
}

/* ---------------- MAIN ---------------- */

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WallFollowerNode>());
    rclcpp::shutdown();
    return 0;
}