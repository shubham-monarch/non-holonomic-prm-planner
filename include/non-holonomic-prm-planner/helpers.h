#ifndef HELPERS_H
#define HELPERS_H

#include <non-holonomic-prm-planner/ds.h>


namespace PRM
{

    

    namespace Utils
    {

        static inline float euclidean(const Node2d &a_, const Node2d &b_)
        {

            return Utils::norm(a_.x_ - b_.x_, a_.y_ - b_.y_);
        }

        static inline float euclidean(const Node3d &a_, const Node3d &b_)
        {

            return Utils::norm(a_.x_ - b_.x_, a_.y_ - b_.y_);
        }

        static Vec3f getNode3dkey(const Node3d &node_)
        {
            return Vec3f{node_.x_, node_.y_, node_.theta_};
        }


    };   
    
    


};



#endif