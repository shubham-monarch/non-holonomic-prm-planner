#ifndef STEERING_CURVE_H
#define STEERING_CURVE_H

#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>

namespace PRM
{

    class SteeringCurve
    {


        public: 

            static geometry_msgs::PoseArray generateSteeringCurve(  geometry_msgs::Pose robot_pose_, const float R_, \
                                                                    const bool trim_  = false, \
                                                                    const float del_sign_ = -1.f, \
                                                                    const float x_dash_ = -1.f);

            
        



        private: 




    };


};



#endif