#include <non-holonomic-prm-planner/visualizations.h>
#include <non-holonomic-prm-planner/simple_prm.h>
#include <non-holonomic-prm-planner/constants.h>
#include <non-holonomic-prm-planner/utils.h>


#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <geometry_msgs/PoseArray.h>

PRM::Visualize::Visualize(){

    ROS_WARN("Inside Visualize Constrcutor!");


}

void PRM::Visualize::draw2DNodes(const std::vector<Node2d> &nodes2d_, std::string topic_)
{

    geometry_msgs::PoseArray sposes_;
    sposes_.header.frame_id = "map";
    sposes_.header.stamp = ros::Time::now(); 
 
    for(const auto &node_ : nodes2d_){

        geometry_msgs::Pose p_;

        const int mx_ = node_.x_, my_ = node_.y_; 
        float wx_, wy_; 
        Utils::mapToWorld(mx_, my_, wx_, wy_);
        
        p_.position.x = wx_; 
        p_.position.y = wy_;
        p_.orientation = Utils::getQuatFromYaw(0.f);
        
        sposes_.poses.push_back(p_);
    
    }

    publishT<geometry_msgs::PoseArray>(topic_, sposes_);
}

void PRM::Visualize::drawCircle(const geometry_msgs::PoseStamped tp_, const float r_, const std::string topic_)
{   

    geometry_msgs::PoseArray circle_;
    circle_.header.frame_id = "map";
    circle_.header.stamp = ros::Time::now(); 

    float sep_ = 5 * M_PI / 180.f;

    for(float theta_ = 0.f; theta_ <= 2 * M_PI; theta_ += sep_)
    {
        
        geometry_msgs::Pose p_; 
        p_.position.x = tp_.pose.position.x + r_ *  std::cos(theta_); 
        p_.position.y = tp_.pose.position.y + r_ *  std::sin(theta_); 
        p_.orientation = Utils::getQuatFromYaw(0.f);

        ROS_INFO("p_: (%f,%f)", p_.position.x, p_.position.y);
        circle_.poses.push_back(p_);
    }

    publishT<geometry_msgs::PoseArray>(topic_, circle_);



}



