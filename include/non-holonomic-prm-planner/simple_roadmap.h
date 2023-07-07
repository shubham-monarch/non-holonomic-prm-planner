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
    
        
    class SimpleRoadmap{

        public:

            SimpleRoadmap(nav_msgs::OccupancyGridConstPtr &map_);
            
            SimpleRoadmap();

            bool samplePoints();
            bool isObstacleFree(const Node &node_) const;     
            void setMapCb(nav_msgs::OccupancyGrid::ConstPtr map_);

            bool plan();

            
        private: 

            int N_ = 1000;

            nav_msgs::OccupancyGridConstPtr grid_;
            geometry_msgs::Pose start_pose_; 
            geometry_msgs::Pose goal_pose_;

            ros::Subscriber map_sub_;
             
            std::vector<Node> nodes_;
            
            //TODO -> change to shared_ptr
            Visualize visualize_;

            bool map_ready_;

            ros::NodeHandle nh_;

            //Graph G; 



    };


};

#endif