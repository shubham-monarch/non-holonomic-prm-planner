#ifndef STEERING_CURVE_H
#define STEERING_CURVE_H




#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>



namespace PRM
{

    struct Node3d; 
        
    class SteeringCurve
    {


        public: 
                        
            static geometry_msgs::PoseArray generateSteeringCurve(  geometry_msgs::Pose robot_pose_, const float R_, \
                                                                    const bool trim_  = false, \
                                                                    const float del_sign_ = -1.f, \
                                                                    const float x_dash_ = -1.f);

            
            static geometry_msgs::PoseArray generateSteeringCurveFamily(const Node3d &node_, \
                                                    std::string topic_ = "family_");
            static geometry_msgs::PoseArray generateSteeringCurveFamily(geometry_msgs::Pose robot_pose_, \
                                                    std::string topic_ = "family_");

            static std::vector<geometry_msgs::PoseStamped> generateSteeringCurveTrimmed(const geometry_msgs::Pose &rp_, \
                                                                                        const geometry_msgs::Pose &cp_);

            static bool connectConfigurationToRobot(    const Node3d &rp_, const Node3d &cp_, \
                                                        const std::string or_topic_, const std::string oc_topic_, \
                                                        const std::string sc_topic_);
            
            static bool connectConfigurationToRobot(   geometry_msgs::Pose or_ , geometry_msgs::Pose oc_, \
                                                const std::string or_topic_, const std::string oc_topic_, \
                                                const std::string sc_topic_); 


        private: 

            

    };


};



#endif