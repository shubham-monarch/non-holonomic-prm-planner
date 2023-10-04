#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>

#include <random>

#include <non-holonomic-prm-planner/sampler.h>
#include <non-holonomic-prm-planner/roadmap.h>

#include <ros/topic.h>

PRM::Sampler::Sampler(const std::string topic): sampled_points_topic_(topic)
{   
    //std::cout << "Sampler constructor called!" << std::endl;
    ROS_DEBUG("Inside sampler constructor!");
    sampled_points_sub_ = nh_.subscribe(sampled_points_topic_, 1, &PRM::Sampler::sampledPointsCallback, this);

    //ros::topic::waitForMessage(sampled_points_topic_, nh_);

    //ros::topic::waitForMessage(sampled_points_topic_, nh_)

    ros::topic::waitForMessage<geometry_msgs::PoseArray>(sampled_points_topic_);

    //ros::topic::waitForMessage    
}

void PRM::Sampler::sampledPointsCallback(geometry_msgs::PoseArrayPtr msg)
{

    ROS_INFO("Inside PRM::Sampler::sampledPointsCallback!");

    sampled_points_.clear();

    std::vector<Node2d> points_;
    points_.reserve(Constants::Planner::N_ + 1000);

    for(const auto &t: msg->poses)
    {   
        const float  x_ = t.position.x;
        const float  y_ = t.position.y;

        const Node2d node_{x_, y_};

        if(sampled_points_.find(node_) == sampled_points_.end())
        {

            sampled_points_.insert(node_);
            //points_.push_back(node_);
    
        }
   

    }

    is_ready  = true; 
     

}

std::vector<PRM::Node2d> PRM::Sampler::generate2DSamplePoints()
{      

    while(ros::ok() && !is_ready)
    {
        ROS_INFO("Waiting for sampled points!");
        ros::spinOnce();
        ros::Duration(0.5).sleep();
        
    }


    ROS_DEBUG("Inside generate2Dsamplepoints function!");
    std::vector<Node2d> points;
    for (const auto &t: sampled_points_)
    {   
        points.push_back(t);
        //ROS_INFO("(x,y) => (%f,%f)", t.x_, t.y_);
    }

     
    geometry_msgs::PoseArray pose_array_;
    pose_array_.header.frame_id = "map"; 
    pose_array_.header.stamp = ros::Time::now(); 



    for(const auto &t: sampled_points_) 
    {
        
        geometry_msgs::Pose pose_; 
        
        pose_.position.x = t.x_;
        pose_.position.y = t.y_; 
        pose_.orientation = Utils::getQuatFromYaw(0.f);

        pose_array_.poses.push_back(pose_);

    }   

    visualize_->publishT<geometry_msgs::PoseArray>("sampled_points", pose_array_);
    
    return points;
}

/*std::vector<PRM::Node2d> PRM::Sampler::generate2DSamplePoints()
{

    //ROS_INFO("Inside simplePRM::samplePoints!");

    ROS_INFO("Sampling of %d points started!", Constants::Planner::N_);
    auto start = std::chrono::high_resolution_clock::now();

    
    const int w_ = Constants::MapMetaData::width_; 
    const int h_ = Constants::MapMetaData::height_;

    ROS_INFO("map_dimension: (%d,%d)", h_, w_);

    //range of x in real world and NOT GRID
    std::vector<float> rx_ = {Constants::MapMetaData::origin_x_ , Constants::MapMetaData::origin_x_ + (w_ + 0.5f) * Constants::MapMetaData::res_}; 
    std::vector<float> ry_ = {Constants::MapMetaData::origin_y_ , Constants::MapMetaData::origin_y_ + (h_ + 0.5f) * Constants::MapMetaData::res_} ;
    
    
    //std::random_device rd;
    unsigned int seed_ = 2; 
    std::mt19937 gen(seed_);
    
    
    std::uniform_real_distribution<float> dist_x(rx_[0] + 100, rx_[0] + 150);
    std::uniform_real_distribution<float> dist_y(ry_[0] + 100, ry_[0] + 150);
     
    int cnt_ =0 ; 

    std::vector<Node2d> points_;
    points_.reserve(Constants::Planner::N_ + 1000);

    while(sampled_points_.size() < Constants::Planner::N_ && ros::ok())  
    {
        //ROS_INFO("nodes2d_.size(): %d", nodes2d_.size());
        const float  x_ = dist_x(gen);
        const float  y_ = dist_y(gen);

        //ROS_INFO("(x,y) => (%f,%f)", x_, y_);

        cnt_++; 
        const Node2d node_{x_, y_};

        if(sampled_points_.find(node_) == sampled_points_.end())
        {

            sampled_points_.insert(node_);
            points_.push_back(node_);

        }
    }

    
    geometry_msgs::PoseArray pose_array_;
    pose_array_.header.frame_id = "map"; 
    pose_array_.header.stamp = ros::Time::now(); 



    for(const auto &t: sampled_points_) 
    {
        
        geometry_msgs::Pose pose_; 
        
        pose_.position.x = t.x_;
        pose_.position.y = t.y_; 
        pose_.orientation = Utils::getQuatFromYaw(0.f);

        pose_array_.poses.push_back(pose_);

    }   

    visualize_->publishT<geometry_msgs::PoseArray>("sampled_points", pose_array_);
    
    return points_; 

}*/
