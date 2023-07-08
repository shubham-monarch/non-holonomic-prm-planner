#ifndef VIS_H
#define VIS_H

#include <ros/ros.h>

#include <non-holonomic-prm-planner/helper.h>
#include <geometry_msgs/PoseArray.h>

namespace PRM{
    
    struct Node; 

    class Visualize {

        public: 
            
            Visualize();

            void visualizeSampledPoints(const std::vector<Node> &nodes_) ;

            void visualizeSteeringCurve(const geometry_msgs::PoseArray &pose_array_);
    



            



        private: 

            ros::NodeHandle nh_;


            ros::Publisher sampled_pts_pub_;
            ros::Publisher steering_curve_pub_;

            //functions
            
    
    };


};



#endif