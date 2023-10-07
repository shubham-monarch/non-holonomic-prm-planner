#ifndef SAMPLER_H
#define SAMPLER_H

#include <unordered_set>
#include <vector>

#include <non-holonomic-prm-planner/ds.h>

#include <geometry_msgs/PoseArray.h>    

namespace PRM
{

    class Sampler
    {
        public: 

            Sampler(const std::string sampled_points_topic);

            //std::vector<Node2d> generate2DSamplePoints();
            //void sampledPointsCallback(geometry_msgs::PoseArrayPtr msg);

            //bool is_ready = false;

            std::vector<Node2d> samplePointsForRowTransition(  const geometry_msgs::PoseStamped &start, 
                                                const geometry_msgs::PoseStamped &goal,
                                                 const int num_points);

        private: 

            std::unordered_set<Node2d, Node2dHash> sampled_points_; 
            
            ros::Subscriber sampled_points_sub_;
            ros::NodeHandle nh_; 
            const std::string sampled_points_topic_;
          
    };


};






#endif