#include <non-holonomic-prm-planner/rrt.h>

PRM::rrt::rrt()
{
    //polygon_ = Polygon();
    //polygon_.outer().push_back(point_t(0, 0));
    //polygon_.outer().push_back(point_t(0, 1));
    //polygon_.outer().push_back(point_t(1, 1));
    //polygon_.outer().push_back(point_t(1, 0)

    start_pose_sub_ = nh_.subscribe("/initialpose", 1, &rrt::initialPoseCb, this);
    goal_pose_sub_ = nh_.subscribe("/goal", 1, &rrt::goalPoseCb, this);
    
}


void PRM::rrt::initialPoseCb(geometry_msgs::PoseWithCovarianceStampedConstPtr pose_)
{

    ROS_WARN("========== START POSE RECEIVED =============="); 

    /*test_start_pose_.header.frame_id= "map" ; 
    test_start_pose_.header.stamp = ros::Time::now(); 

    test_start_pose_.pose.position.x = pose_->pose.pose.position.x; 
    test_start_pose_.pose.position.y = pose_->pose.pose.position.y;
    test_start_pose_.pose.orientation = pose_->pose.pose.orientation;   

    start_pose_set_ = true; 

    visualize_->publishT<geometry_msgs::PoseStamped>("test_start_pose", test_start_pose_, true);
    */
    return; 
}

void PRM::rrt::goalPoseCb(geometry_msgs::PoseStampedConstPtr pose_)
{
    ROS_WARN("========== GOAL POSE RECEIVED =============="); 
    
    /*test_goal_pose_ = *pose_;

    goal_pose_set_ = true; 

    visualize_->publishT<geometry_msgs::PoseStamped>("test_goal_pose", test_goal_pose_, true);

    if(!start_pose_set_)
    {
        ROS_ERROR("start_pose is not set!");
        return;
    }

    bool flag_ = plan(test_start_pose_, test_goal_pose_);

    if(flag_)
    {
        ROS_WARN("======= PLAN WAS FOUND ======");
        Roadmap::success_cnt_++;
    }   
    */

   
    return;

}   




