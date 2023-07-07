#include <non-holonomic-prm-planner/planner.h>
#include <non-holonomic-prm-planner/simple_roadmap.h>

PRM::Planner::Planner():map_ready_{false}
{

    
    map_sub_ = nh_.subscribe(Constants::map_topic, 1, &Planner::setMapCb, this);

    ros::Rate r(10);
    
    while(!map_ready_) {
        
        ROS_DEBUG("Map is not ready!");
        ros::spinOnce();
        r.sleep();
    
    }

    start_pose_sub_ = nh_.subscribe("/initialpose", 1, &Planner::setStartCb, this);
    goal_pose_sub_ = nh_.subscribe("/goal", 1, &Planner::setGoalCb, this);
    
    

}

void PRM::Planner::setMapCb(nav_msgs::OccupancyGrid::ConstPtr map_)
{

    Constants::MapMetaData::origin_x_ = map_->info.origin.position.x;
    Constants::MapMetaData::origin_y_ = map_->info.origin.position.y;
    Constants::MapMetaData::cell_size_ = map_->info.resolution;
    Constants::MapMetaData::height_ = map_->info.height;
    Constants::MapMetaData::width_ = map_->info.width;
    
    grid_ = map_;

    ROS_DEBUG("MAP PARAMS ===>");
    ROS_DEBUG("dimensions: (%d, %d)", map_->info.height, map_->info.width);
    ROS_DEBUG("resolution: %f", map_->info.resolution);
    ROS_DEBUG("origin: (%f, %f)", map_->info.origin.position.x, map_->info.origin.position.y);
    
    map_ready_ = true;

    
}
            

void PRM::Planner::setStartCb(const geometry_msgs::PoseWithCovarianceStampedConstPtr &initial)
{
   
    latest_start_pose_ = *initial;

    ROS_DEBUG("initial_pose: (%f,%f)", initial->pose.pose.position.x, initial->pose.pose.position.y);

}

  
void PRM::Planner::setGoalCb(const geometry_msgs::PoseStampedConstPtr &end)
{
    
    latest_goal_pose_ = *end;

    ROS_DEBUG("goal_pose_: (%f,%f)", end->pose.position.x, end->pose.position.y);

}
   

