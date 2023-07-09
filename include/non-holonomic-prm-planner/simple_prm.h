#ifndef PRM_PLANNER_H
#define PRM_PLANNER_H

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>

#include <non-holonomic-prm-planner/visualizations.h>

/**
 * Simple PRM implementation
*/


//SubRoutines ==> 
//Sampling
//Resampling
//Local planner
//CollisionDetection ***
//Distance Metrics
//k-Nearest neighbours


namespace PRM{
    
    class Visualize;

    //represents a sampled point with no heading information
    struct Node2d {

        int x_, y_; 
        float theta_;

        Node2d(const int x, const int y) : x_(x), y_(y){}
    
        
    };
    
        
    class SimplePRM{

        public:

       
            SimplePRM();

            
            void setMapCb(nav_msgs::OccupancyGrid::ConstPtr map_);
            bool isMapSet();

            void initialize();
            bool plan();
            
        private: 


            // ===== functions ====

            bool generateSamplePoints();
            bool isObstacleFree(const Node2d &node_) const;     
            
            bool generateSteeringCurve(geometry_msgs::Pose robot_pose_,  float delta_);

            bool generateSteeringCurve(geometry_msgs::Pose robot_pose_,  geometry_msgs::Pose &config_pose_, float R_);
            
            
            void generateSteeringCurveFamily(geometry_msgs::Pose robot_pose_);
            
            bool connectConfigurationToRobot(geometry_msgs::Pose rp_, geometry_msgs::Pose configuration_)   ;


            //==== variables =====
            Visualize visualize_;
            bool map_set_;
            nav_msgs::OccupancyGridConstPtr grid_;



            //** planner tuning params
            static const int N_ = 10;  //number of initial poses
            
            
            //core planner vars
            std::vector<Node2d> nodes2d_;
            std::vector<geometry_msgs::Pose> steering_curve_family_poses_;


            //** ROS members
            ros::NodeHandle nh_;
            ros::Subscriber map_sub_;
            


    };


};

#endif