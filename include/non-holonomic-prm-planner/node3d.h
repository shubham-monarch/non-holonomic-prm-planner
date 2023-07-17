#ifndef NODE3D_H
#define NODE3D_H


//#include <non-holonomic-prm-planner/ds.h>
#include <non-holonomic-prm-planner/constants.h>

#include <ros/ros.h>
    

namespace PRM
{

    struct Edge ;
    
    class Node3d
    {
        public:

            const float x_, y_;  //world co-ordinates  
            const int theta_idx_;  // theta_ = (theta_) 
            const float theta_;    //world heading
            


            Node3d():x_(-1.0), y_(1.f), theta_idx_(0), theta_(-1){};
        
            Node3d(const float x, const float y, const int idx_):x_(x), y_(y), \
                            theta_idx_(idx_), \
                            theta_(1.f * theta_idx_ * Constants::Planner::theta_sep_ )
            {

            }

            bool operator()(const Node3d& p1, const Node3d& p2) const {
                // Compare based on the distance from the origin (0,0,0)
                //double dist1 = p1.x * p1.x + p1.y * p1.y + p1.z * p1.z;
                //double dist2 = p2.x * p2.x + p2.y * p2.y + p2.z * p2.z;
                return p1.cost_ > p2.cost_; // Greater-than comparison for min heap
            }

            bool operator==(const Node3d& other_) const 
            {
                return x_ == other_.x_ && y_ == other_.y_ && theta_idx_ == other_.theta_idx_;
            }   

        

            void print() const
            {
                ROS_INFO("========================");
                ROS_INFO("Node3d ==> (%f,%f,%d, %f)", x_, y_, theta_idx_, theta_);
                ROS_INFO("========================");
                
            }

        private: 

            float cost_;
            std::shared_ptr<std::vector<Edge> > edges_;  //pointer to node edges



            bool addEdge(); 
            bool updateCost();
      
    };

};




#endif