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

            bool samplePoints();
            bool isObstacleFree(const Node &node_) const;     

            void setMapCb(nav_msgs::OccupancyGrid::ConstPtr map_);
            bool isMapSet();

            void initialize();

            bool plan();
            
        private: 


            Visualize visualize_;
            bool map_set_;
            nav_msgs::OccupancyGridConstPtr grid_;


            //** planner params
            int N_ = 1000;
            const float delta_max_ = 40 *M_PI / 180.0;  //max steering angle in radians
            const float a_2_ = 0.8; //len_to_hitch 
            const float l_ = 1.84; //wheelbase
            float r_min_;  //minimum turning radius
            float max_res_; // max allowed resolution


            //** planner variables
            std::vector<Node> nodes_;


            //** ROS members
            ros::NodeHandle nh_;
            ros::Subscriber map_sub_;
            


    };


};

#endif