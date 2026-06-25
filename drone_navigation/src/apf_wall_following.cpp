#include "rclcpp/rclcpp.hpp"
#include "Eigen/Dense"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include <std_msgs/msg/bool.hpp>
#include "tf2_msgs/msg/tf_message.hpp"
#include <cmath>

using namespace std::placeholders;
using namespace std::chrono_literals;

class APFNode : public rclcpp::Node 
{
public:
    APFNode() : Node("apf_node")
    {
        pose_subscriber_ = create_subscription<tf2_msgs::msg::TFMessage>("/tf", 10, 
                                                                          std::bind(&APFNode::callback_pose, this, _1));
        obstacle_subscriber_ = create_subscription<geometry_msgs::msg::PoseArray>("/obstacles/poses", 10, 
                                                                          std::bind(&APFNode::callback_obs_sub, this, _1));
        is_wall_subscriber_ = create_subscription<std_msgs::msg::Bool>("/wall_follow_active", 10, 
                                                                          std::bind(&APFNode::is_wall_callback, this, _1));
        main_timer_ = create_wall_timer(0.1s, std::bind(&APFNode::main_loop, this));
        initialisation_timer_ = create_wall_timer(0.1s, std::bind(&APFNode::initialisation, this));
        wp_publisher_ = create_publisher<geometry_msgs::msg::Point>("/px4_0/waypoint", 10);

        this->declare_parameter("threat_dist", 0.5);
        this->declare_parameter("K_att", 10.0);
        this->declare_parameter("K_rep", 2.0);
        this->declare_parameter("wp_dist", 0.4);
        this->declare_parameter("goal_x", 4.0);
        this->declare_parameter("goal_y", 1.0);
        threat_dist_ = this->get_parameter("threat_dist").as_double();
        K_att_ = this->get_parameter("K_att").as_double();
        K_rep_= this->get_parameter("K_rep").as_double();
        wp_dist_= this->get_parameter("wp_dist").as_double();
        double goal_x= this->get_parameter("goal_x").as_double();
        double goal_y= this->get_parameter("goal_y").as_double();
        
        target_pos_ = Eigen::Vector2d(goal_x, goal_y);

    }

private:

    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr pose_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr obstacle_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr is_wall_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr wp_publisher_;
    rclcpp::TimerBase::SharedPtr main_timer_;
    rclcpp::TimerBase::SharedPtr initialisation_timer_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    tf2_msgs::msg::TFMessage pose_msg_;
    geometry_msgs::msg::PoseArray obstacles_;

    Eigen::Vector2d curr_pos_;
    Eigen::Vector2d target_pos_;
    Eigen::Vector2d curr_wp_ = target_pos_;

    bool has_pose_ = false;
    bool has_obstacles_ = false;
    bool is_initialised_ = false;
    bool is_at_goal_ = false;
    bool is_wall_ = false;

    double curr_yaw_;
    double curr_z_;
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


    void callback_pose(tf2_msgs::msg::TFMessage::SharedPtr msg){
        pose_msg_ = *msg;
        auto transforms = pose_msg_.transforms;
        double curr_x = transforms[0].transform.translation.x;
        double curr_y = transforms[0].transform.translation.y;
        curr_z_ = transforms[0].transform.translation.z;
        curr_pos_ = Eigen::Vector2d(curr_x, curr_y);
        
        double q_x = transforms[0].transform.rotation.x;
        double q_y = transforms[0].transform.rotation.y;
        double q_z = transforms[0].transform.rotation.z;
        double q_w = transforms[0].transform.rotation.w;

        double siny_cosp = 2.0 * (q_w * q_z + q_x * q_y);
        double cosy_cosp = 1.0 - 2.0 * (q_y * q_y + q_z * q_z);
                
        curr_yaw_ = std::atan2(siny_cosp, cosy_cosp);
        has_pose_ = true;   
    }

    void callback_obs_sub(geometry_msgs::msg::PoseArray::SharedPtr msg){
        obstacles_ = *msg;
        has_obstacles_ = true;
    }

    void initialisation(){
        auto waypoint = geometry_msgs::msg::Point();
        waypoint.x = 0;
        waypoint.y = 0;
        waypoint.z = 5.0;

        wp_publisher_->publish(waypoint);

        RCLCPP_INFO(this->get_logger(), "Initialisation in process... ");
        
        if(abs(curr_z_ - 5.0) < 0.5){
            is_initialised_ = true;
            initialisation_timer_->cancel();
        }
    }

    void is_wall_callback(std_msgs::msg::Bool::SharedPtr msg){
        is_wall_ = msg->data;

    }

    void main_loop(){
        if(!has_pose_ || !has_obstacles_ || !is_initialised_){
            RCLCPP_INFO(this->get_logger(), "Insufficient data. Aborting!!!");
            return;
        }

        if (is_wall_){
            RCLCPP_INFO(this->get_logger(), "Wall detected, switching to wall following");
            return;
        }

        // check if robot has reached goal, if yes, break
        double dist_err = distance(curr_pos_, target_pos_);
        if (dist_err < 0.2) {
            is_at_goal_ = true;
            RCLCPP_INFO(this->get_logger(), "curr x = %f, curr y = %f", curr_pos_.x(), curr_pos_.y());
            RCLCPP_INFO(this->get_logger(), "Goal reached");
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
                // Eigen::Vector2d o_P_obs = curr_pos_ + o_R_m*m_P_obs;
                Eigen::Vector2d o_P_obs = curr_pos_ + m_P_obs;  // the rotation matrix is not used because the costmap doesn't rotate with the drone
                F_rep += force_repulsive(o_P_obs);
            }
            
        }

        Eigen::Vector2d F_all = F_att + F_rep;
        // RCLCPP_INFO(this->get_logger(), "Calculated resultant force");
        RCLCPP_INFO(this->get_logger(), "F_all: %f", F_all.norm());
        RCLCPP_INFO(this->get_logger(), "F_att: %f", F_att.norm());
        RCLCPP_INFO(this->get_logger(), "F_rep: %f", F_rep.norm());


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

        //publish waypoint
        auto waypoint = geometry_msgs::msg::Point();
        waypoint.x = curr_wp_.x();
        waypoint.y = curr_wp_.y();
        waypoint.z = 5.0;

        wp_publisher_->publish(waypoint);



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