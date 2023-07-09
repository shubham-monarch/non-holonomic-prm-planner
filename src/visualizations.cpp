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

    sampled_pts_pub_ = nh_.advertise<geometry_msgs::PoseArray>("sampled_points",   1, true);
    steering_curve_pub_ = nh_.advertise<geometry_msgs::PoseArray>("steering_curve", 1, true);
    point_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("point_pose", 1, true);
    
    ROS_WARN("Inside Visualize Constrcutor!");


}

void PRM::Visualize::visualizeSampledPoints(const geometry_msgs::PoseArray &poses_) {


    sampled_pts_pub_.publish(poses_);
    
}

void PRM::Visualize::visualizeSteeringCurve(const geometry_msgs::PoseArray &pose_array_)
{
    steering_curve_pub_.publish(pose_array_);

}

void PRM::Visualize::visualizePointPose(const geometry_msgs::PoseStamped &pose_)
{

    point_pose_pub_.publish(pose_);

}