#include "rclcpp/rclcpp.hpp"
#include "Eigen/Dense"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

using namespace std::placeholders;
using namespace std::chrono_literals;

class APFNode : public rclcpp::Node 
{
public:
    APFNode() : Node("apf_node")
    {
        pose_subscriber_ = create_subscription<nav_msgs::msg::Odometry>("/odom", 10, 
                                                                          std::bind(&APFNode::callback_odom, this, _1));
        obstacle_subscriber_ = create_subscription<geometry_msgs::msg::PoseArray>("/obstacles/poses", 10, 
                                                                          std::bind(&APFNode::callback_obs_sub, this, _1));
        main_timer_ = create_wall_timer(0.1s, std::bind(&APFNode::main_loop, this));
        control_timer_ = create_wall_timer(0.1s, std::bind(&APFNode::control_loop, this));
        velocity_publisher_ = create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel", 10);

        this->declare_parameter("threat_dist", 0.5);
        this->declare_parameter("K_att", 10.0);
        this->declare_parameter("K_rep", 2.0);
        this->declare_parameter("wp_dist", 0.4);
        threat_dist_ = this->get_parameter("threat_dist").as_double();
        K_att_ = this->get_parameter("threat_dist").as_double();
        K_rep_= this->get_parameter("threat_dist").as_double();
        wp_dist_= this->get_parameter("wp_dist").as_double();
    }

private:

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pose_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr obstacle_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_publisher_;
    rclcpp::TimerBase::SharedPtr main_timer_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    nav_msgs::msg::Odometry odom_msg_;
    geometry_msgs::msg::PoseArray obstacles_;

    Eigen::Vector2d curr_pos_;
    Eigen::Vector2d target_pos_ = Eigen::Vector2d(4.0, 1.0);
    Eigen::Vector2d curr_wp_ = target_pos_;

    bool has_odom_ = false;
    bool has_obstacles_ = false;
    bool is_at_goal_ = false;

    double curr_yaw_;
    double K_att_;
    double K_rep_;
    double threat_dist_; // in meters
    double wp_dist_; // distance to travel in the newly calculated path. 
                           //TODO: Consider making this distance proportional to the magnitude of the resultant force
    double apf_vel_;
    double max_vel_ = 0.4; // Maximum velocity of the drone

    //P-Controller
    double Kp_lin_ = 1.0;
    double Kp_ang_ = 1.0;


    void callback_odom(nav_msgs::msg::Odometry::SharedPtr msg){
        odom_msg_ = *msg;
        double curr_x = odom_msg_.pose.pose.position.x;
        double curr_y = odom_msg_.pose.pose.position.y;
        curr_pos_ = Eigen::Vector2d(curr_x, curr_y);
        
        double q_z = odom_msg_.pose.pose.orientation.z;
        double q_w = odom_msg_.pose.pose.orientation.w;
        
        curr_yaw_ = 2.0 * std::atan2(q_z, q_w);
        has_odom_ = true;   
    }

    void callback_obs_sub(geometry_msgs::msg::PoseArray::SharedPtr msg){
        obstacles_ = *msg;
        has_obstacles_ = true;
    }

    void main_loop(){
        if(!has_odom_ || !has_obstacles_){
            return;
        }

        // check if robot has reached goal, if yes, break
        double dist_err = distance(curr_pos_, target_pos_);
        if (dist_err < 0.2) {
            is_at_goal_ = true;
            return;
        }


        Eigen::Vector2d F_att = force_attractive();
        Eigen::Vector2d F_rep;

        Eigen::Matrix2d o_R_m;
        o_R_m << cos(curr_yaw_), -sin(curr_yaw_),
                sin(curr_yaw_), cos(curr_yaw_);

        if(obstacles_.poses.empty()){
            F_rep = Eigen::Vector2d(0, 0);
        }else{
            F_rep = Eigen::Vector2d(0, 0);
            for(const auto& obstacle : obstacles_.poses){
                Eigen::Vector2d m_P_obs(obstacle.position.x, obstacle.position.y);
                Eigen::Vector2d o_P_obs = curr_pos_ + o_R_m*m_P_obs;
                F_rep += force_repulsive(o_P_obs);
            }
            RCLCPP_INFO(this->get_logger(), "Calculated resultant force");
        }

        Eigen::Vector2d F_all = F_att + F_rep;

        // To generate a new path from the resultant force
        // Obtain magnitude and direction of resultant force in absolute frame
        double F_all_mag = F_all.norm();
        double F_all_angle = std::atan2(F_all.y(), F_all.x());
        
        //Transform to robot frame
        double alpha = F_all_angle - curr_yaw_;
        double m_x_new = wp_dist_*cos(alpha);
        double m_y_new = wp_dist_*sin(alpha);
        Eigen::Vector2d m_P_new(m_x_new, m_y_new);

        //Transform point to absolute frame
        Eigen::Vector2d o_P_new = curr_pos_ + o_R_m*m_P_new;
        curr_wp_ = o_P_new;
        apf_vel_ = F_all_mag;

    }

    void control_loop(){
        if(!has_odom_){
            return;
        }

        double dist_err = distance(curr_pos_, curr_wp_);
        double angle_to_goal = std::atan2(curr_wp_.y() - curr_pos_.y(), curr_wp_.x() - curr_pos_.x());
        
        
        double angle_error = angle_to_goal - curr_yaw_;
        
        // Normalize to [-pi, pi] using atan2(sin, cos) - very robust
        angle_error = std::atan2(std::sin(angle_error), std::cos(angle_error));

        
        geometry_msgs::msg::TwistStamped vel_msg;

        if (dist_err < 0.2 || is_at_goal_) {
            vel_msg.twist.linear.x = 0.0;
            vel_msg.twist.angular.z = 0.0;
            RCLCPP_INFO(this->get_logger(), "Goal Reached!");
        } else {
        
            vel_msg.twist.linear.x = std::min({max_vel_, apf_vel_, Kp_lin_ * dist_err}); 
            vel_msg.twist.angular.z = Kp_ang_ * angle_error;
            RCLCPP_INFO(this->get_logger(), "Heading to current waypoint");
        }
    
        velocity_publisher_->publish(vel_msg);
    }

    Eigen::Vector2d force_attractive(){
        double d_dx = -(target_pos_.x() - curr_pos_.x())/distance(curr_pos_, target_pos_);
        double d_dy = -(target_pos_.y() - curr_pos_.y())/distance(curr_pos_, target_pos_);
        Eigen::Vector2d d_dX(d_dx, d_dy);
        Eigen::Vector2d F_att = -2*K_att_*distance(curr_pos_, target_pos_)*d_dX;
        return F_att;

    }

    Eigen::Vector2d force_repulsive(Eigen::Vector2d obs_pos){
        double obs_dist = distance(curr_pos_, obs_pos);
        double d_dx = -(obs_pos.x() - curr_pos_.x())/obs_dist;
        double d_dy = -(obs_pos.y() - curr_pos_.y())/obs_dist;
        Eigen::Vector2d d_dX(d_dx, d_dy);
        Eigen::Vector2d F_rep;
        if(obs_dist <= threat_dist_){
            F_rep = 2*K_rep_*((1/obs_dist) - (1/threat_dist_))*(1/(obs_dist*obs_dist))*d_dX;
        }else{
            F_rep << 0, 0;
        }
        return F_rep;

    }

    double distance(Eigen::Vector2d start, Eigen::Vector2d end){
        Eigen::Vector2d start_to_end = end - start;
        double distance = start_to_end.norm();
        return distance;
    }

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<APFNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}