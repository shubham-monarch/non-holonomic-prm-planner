#ifndef DS_H
#define DS_H

#include <boost/functional/hash.hpp>
#include <ros/ros.h>

#include <Eigen/Dense>
#include <eigen3/Eigen/Core>

#include <non-holonomic-prm-planner/constants.h>    
#include <non-holonomic-prm-planner/utils.h>
#include <non-holonomic-prm-planner/node3d.h>



namespace PRM
{   

    struct Node3d;  //forward declaration

    //tils::Eigen::Matrix3f getHomogeneousTransformationMatrix(const Eigen::Vector2f &translation, const float &theta) ;
        
    typedef Eigen::Matrix3f Mat3f;
    typedef Eigen::Vector2f Vec2f;
    typedef long long int ll;



    struct Node2d 
    {

        const float x_, y_; 
    
        Node2d(const float  x, const float y) : x_(x), y_(y){}
        
        ///Node2d(): x_(0.f), y_(0.f) {}

        bool operator==(const Node2d& other) const 
        {
            return x_ == other.x_ && y_ == other.y_;
        }    
    };

    struct Node2dHash 
    {
        std::size_t operator()(const Node2d& obj) const {
            std::size_t seed = 0;
            boost::hash_combine(seed, obj.x_);
            boost::hash_combine(seed, obj.y_);
            return seed;
        }
    };

    struct Edge
    {
        
        explicit Edge(const Node3d &node, const float dc, const float ac):  node_(std::make_shared<Node3d const>(node)), \
                                                                            dc_(dc),    \
                                                                            ac_(ac),    \
                                                                            tc_(ac_ + dc_)
                                                                                            
        {}

        Edge() {};

        void print() const
        {

            ROS_WARN("========================== EDGE =============================="); 

            ROS_INFO("destination node ===> (%f,%f,%f)", node_->x_, node_->y_, node_->theta_);
            ROS_INFO("(dc_, ac_, tc_) ==> (%f,%f,%f)", dc_, ac_, tc_);
            ROS_INFO("========================== EDGE =============================="); 

        }
        
        float dc_;    //distance cost 
            
          
        private:
        
            std::shared_ptr<Node3d const> node_; //destination node

            float ac_ ;   //angular cost
            float tc_ ;   // total cost 

        
    };


};


#endif