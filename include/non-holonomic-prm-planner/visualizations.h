#ifndef VIS_H
#define VIS_H

#include <ros/ros.h>

//#include <non-holonomic-prm-planner/simple_roadmap.h>
#include <non-holonomic-prm-planner/helper.h>
#include <non-holonomic-prm-planner/constants.h>


namespace PRM{
    
    struct Node; 

    class Visualize {

        public: 
            
            Visualize();

            void visualizeSampledPoints(const std::vector<Node> &nodes_) ;







        private: 

            ros::NodeHandle nh_;


            ros::Publisher sampled_pts_pub_;
    };


};



#endif