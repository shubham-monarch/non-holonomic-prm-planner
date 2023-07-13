#ifndef DS_H
#define DS_H

#include <boost/functional/hash.hpp>
#include <ros/ros.h>

#include <Eigen/Dense>
#include <eigen3/Eigen/Core>

#include <non-holonomic-prm-planner/constants.h>    
#include <non-holonomic-prm-planner/utils.h>


namespace PRM
{   


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




    
   struct Node3d 
   {

        const float x_, y_;  //world co-ordinates  
        const int theta_idx_;  // theta_ = (theta_) 
        const float theta_;    //world heading
        

        Node3d(const float x, const float y, const int idx_):x_(x), y_(y), \
                        theta_idx_(idx_), \
                        theta_(1.f * theta_idx_ * Constants::Planner::theta_sep_ )
        {

        }
                                                                
        bool operator==(const Node3d& other_) const 
        {
            return x_ == other_.x_ && y_ == other_.y_ && theta_idx_ == other_.theta_idx_;
        }   

    };

    

    struct Node3dHash 
    {
        std::size_t operator()(const Node3d& obj) const {
            std::size_t seed = 0;
            boost::hash_combine(seed, obj.x_);
            boost::hash_combine(seed, obj.y_);
            boost::hash_combine(seed, obj.theta_idx_);
            return seed;
        }
    };
    

    //TODO ==> convert node2d to node2d*
    //directed edge from p1 -> p2
    struct Edge
    {

        const Node3d a_;
        const Node3d b_;

        private: 

            float dc_;     //distance traversal cost
            float ac_;      //angular cost

            float cost_;          //cost_ = w_dc_ * dc_ + w_ac_ * ac_ ;
            

        explicit Edge(const Node3d &n1, const Node3d &n2): a_(n1) , b_(n2)
        {
            
            const float theta_a_ = 1.f * a_.theta_idx_ * Constants::Planner::theta_sep_;
            const float theta_b_ = 1.f * b_.theta_idx_ * Constants::Planner::theta_sep_;
            
            if(std::fabs(theta_a_ - a_.theta_) > 0.01 || std::fabs(theta_b_ - b_.theta_) > 0.01) 
            {
                ROS_ERROR("theta_a_ != a_.theta_ ==> Something is wrong!");
            }

            const Vec2f V_oa_(a_.x_, a_.y_); 
            const Vec2f V_ob_(b_.x_, b_.y_); 
            

            const Mat3f &P_oa_ = (Utils::getHomogeneousTransformationMatrix(V_oa_, theta_a_));      //pose of a in origin frame
            //const Mat3f &P_ob_ = (Utils::getHomogeneousTransformationMatrix(V_ob_, theta_b_));      //pose of b in origin frame
            
            //const Mat3f &P_ao_ = P_oa_.inverse();           //pose of o in a

            //const Mat3f &P_ab_ = P_ao_ * P_ob_;             //pose of b in a

            //float theta_dash_  = std::atan2(P_ab_(1,0), P_ab_(0,0));


        };


        
    };


};


#endif