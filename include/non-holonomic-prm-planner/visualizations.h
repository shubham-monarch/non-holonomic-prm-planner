#ifndef VIS_H
#define VIS_H

#include <ros/ros.h>

#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>

namespace PRM{
    
    struct Node2d; 

    class Visualize {

        public: 
            
            Visualize();

            void visualizeSampledPoints(const std::vector<Node2d> &nodes2d_) ;

            void visualizeSteeringCurve(const geometry_msgs::PoseArray &pose_array_) ;
    
            void visualizePointPose(const geometry_msgs::PoseStamped &point_) ;


            



        private: 

            ros::NodeHandle nh_;


            ros::Publisher sampled_pts_pub_;
            ros::Publisher steering_curve_pub_;
            ros::Publisher point_pose_pub_;

            //functions
            
    
    };


};



#endif