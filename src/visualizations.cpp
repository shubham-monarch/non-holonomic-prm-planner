#include <non-holonomic-prm-planner/visualizations.h>
#include <non-holonomic-prm-planner/simple_prm.h>
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

void PRM::Visualize::drawNodeNeighbours(const NodePtr_ &node_, const std::string topic_)
{
    
    geometry_msgs::PoseArray init_pose_neighbours_;
    init_pose_neighbours_.header.frame_id = "map"; 
    init_pose_neighbours_.header.stamp = ros::Time::now(); 
    int sz_ = (*node_->edges_).size();

    //ROS_INFO("sz_: %d", sz_);
    
    /*if(sz_ > 1)
    {
        ROS_DEBUG("Source Node ==>");
        node_->print(); 
        ROS_DEBUG("Destination Nddes ==>");
    
    }*/
    for(const auto edge_: *node_->edges_)
    {   
        if(sz_ > 1)
        {
            //edge_.node_->print();
      
        }
        //node_->print();
        geometry_msgs::Pose p_; 
        p_.position.x = edge_.node_->x_ ; 
        p_.position.y = edge_.node_->y_ ; 
        p_.orientation = Utils::getQuatFromYaw(edge_.node_->theta_) ; 

        init_pose_neighbours_.poses.push_back(p_);
    }
    
    //ROS_INFO("sz_: %d", sz_);
    //if(sz_ > 1) ROS_WARN("neighbours.size(): %d", init_pose_neighbours_.poses.size());

    publishT<geometry_msgs::PoseArray>(topic_, init_pose_neighbours_);

}

void PRM::Visualize::draw3DNodes(const std::vector<Node3d> &nodes3d_, std::string topic_)
{

    geometry_msgs::PoseArray sposes_;
    sposes_.header.frame_id = "map";
    sposes_.header.stamp = ros::Time::now(); 
 
    for(const auto &node_ : nodes3d_){

        geometry_msgs::Pose p_;

        p_.position.x = node_.x_; 
        p_.position.y = node_.y_;
        p_.orientation = Utils::getQuatFromYaw(node_.theta_);
        
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

void PRM::Visualize::drawPoint(const float x_, const float y_, const std::string topic_)
{

    geometry_msgs::PoseStamped  pose_; 
    pose_.header.frame_id = "map"; 
    pose_.header.stamp = ros::Time::now(); 

    pose_.pose.position.x= x_; 
    pose_.pose.position.y = y_; 
    pose_.pose.orientation = Utils::getQuatFromYaw(0.f);

    publishT(topic_, pose_);

}

void PRM::Visualize::drawNode3d(const Node3d &node_, std::string topic_)
{
    geometry_msgs::PoseStamped  pose_; 
    pose_.header.frame_id = "map"; 
    pose_.header.stamp = ros::Time::now(); 

    pose_.pose.position.x   = node_.x_; 
    pose_.pose.position.y   = node_.y_; 
    pose_.pose.orientation = Utils::getQuatFromYaw(node_.theta_);

    publishT(topic_, pose_);

}


void PRM::Visualize::drawPoint(const geometry_msgs::Pose &p_, const std::string topic_)
{

    geometry_msgs::PoseStamped  pose_; 
    pose_.header.frame_id = "map"; 
    pose_.header.stamp = ros::Time::now(); 

    pose_.pose.position.x = p_.position.x; 
    pose_.pose.position.y = p_.position.y;  
    pose_.pose.orientation = p_.orientation;

    publishT(topic_, pose_);

}
