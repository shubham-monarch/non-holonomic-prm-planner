#ifndef DS_H
#define DS_H

#include <boost/functional/hash.hpp>
#include <non-holonomic-prm-planner/constants.h>


namespace PRM
{

    struct Node2d {

        const float x_, y_; 
    
        explicit Node2d(const float  x, const float y) : x_(x), y_(y){}
        
        ///Node2d(): x_(0.f), y_(0.f) {}

        bool operator==(const Node2d& other) const {
            return x_ == other.x_ && y_ == other.y_;
        }    
    };

    struct Node2dHash {
        std::size_t operator()(const Node2d& obj) const {
            std::size_t seed = 0;
            boost::hash_combine(seed, obj.x_);
            boost::hash_combine(seed, obj.y_);
            return seed;
        }
    };




    
   struct Node3d {

        const float x_, y_;  //world co-ordinates  
        const int theta_idx_;  // theta_ = (theta_) 
        const float theta_;    //world heading
        

        explicit Node3d(const float x, const float y, const int idx_):x_(x), y_(y), \
                        theta_idx_(idx_), \
                        theta_(1.f * theta_idx_ * Constants::Planner::theta_sep_ )
        {

        }
                                                                
        bool operator==(const Node3d& other_) const {
            return x_ == other_.x_ && y_ == other_.y_ && theta_idx_ == other_.theta_idx_;
        }   

    };

    

    struct Node3dHash {
        std::size_t operator()(const Node3d& obj) const {
            std::size_t seed = 0;
            boost::hash_combine(seed, obj.x_);
            boost::hash_combine(seed, obj.y_);
            boost::hash_combine(seed, obj.theta_idx_);
            return seed;
        }
    };
    



};


#endif