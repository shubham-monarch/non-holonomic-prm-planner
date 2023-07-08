#ifndef PRM_PLANNER_H
#define PRM_PLANNER_H

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>

#include <non-holonomic-prm-planner/visualizations.h>

/**
 * Simple PRM implementation
*/

namespace PRM{
    
    class Visualize;

    struct Node {

        int x_, y_; 
        float theta_;

        Node(const int x, const int y, const float theta) : x_(x), y_(y), theta_(theta){}
    };
    
        
    class SimplePRM{

        public:

       
            SimplePRM();

            bool generateSamplePoints();
            bool isObstacleFree(const Node &node_) const;     

            void setMapCb(nav_msgs::OccupancyGrid::ConstPtr map_);
            bool isMapSet();

            void initialize();
            bool plan();
            
        private: 


            //**  functions
            void initializeVehicleParams();
            bool generateSteeringCurve(geometry_msgs::Pose robot_pose_,  float delta_);

            void generateSteeringCurveFamily(geometry_msgs::Pose robot_pose_);
        


            //*** variables

            Visualize visualize_;
            bool map_set_;
            nav_msgs::OccupancyGridConstPtr grid_;



            //** planner variables
            static const int N_ = 10;  //number of initial poses
            std::vector<Node> nodes_;


            //** ROS members
            ros::NodeHandle nh_;
            ros::Subscriber map_sub_;
            
            std::vector<geometry_msgs::Pose> steering_curve_family_poses_;

            //symbols used
            //δ ==> delta ==> steering angle


    };


};

#endif