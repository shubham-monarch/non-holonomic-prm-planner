#ifndef PRM_H
#define PRM_H

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <tf/transform_datatypes.h>
#include <std_msgs/String.h>

namespace PRM{

    class Planner{

        public:

            Planner();
            
            
            //~Planner();

        protected: 



        private:

            //void setMapCb(const nav_msgs::OccupancyGridConstPtr &map_);
            void setMapCb(nav_msgs::OccupancyGridConstPtr map_);

            void setStartCb(const geometry_msgs::PoseWithCovarianceStampedConstPtr &initial);
            void setGoalCb(const geometry_msgs::PoseStampedConstPtr &end);
  

            ros::NodeHandle nh_;
            ros::Subscriber map_sub_;

            ros::Subscriber start_pose_sub_, goal_pose_sub_;


            bool map_ready_;

            geometry_msgs::PoseWithCovarianceStamped latest_start_pose_;
            geometry_msgs::PoseStamped latest_goal_pose_;
  

        
    };
    




};

#endif