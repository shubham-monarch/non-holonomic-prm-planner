#include <non-holonomic-prm-planner/planner.h>
#include <non-holonomic-prm-planner/constants.h>

PRM::Planner::Planner():map_ready_{false}
{



    map_sub_ = nh_.subscribe(Constants::map_topic, 1, &Planner::setMapCb, this);

    ros::Rate r(10);
    
    while(!map_ready_) {
        
        ros::spinOnce();
        r.sleep();
    

    }
    start_pose_sub_ = nh_.subscribe("/initialpose", 1, &Planner::setStartCb, this);
    goal_pose_sub_ = nh_.subscribe("/goal", 1, &Planner::setGoalCb, this);
    


}

void PRM::Planner::setMapCb(nav_msgs::OccupancyGrid::ConstPtr map_)
{



}
            

void PRM::Planner::setStartCb(const geometry_msgs::PoseWithCovarianceStampedConstPtr &initial)
{
   
    latest_start_pose_ = *initial;

}

  
void PRM::Planner::setGoalCb(const geometry_msgs::PoseStampedConstPtr &end)
{
    
    latest_goal_pose_ = *end;

}
   

