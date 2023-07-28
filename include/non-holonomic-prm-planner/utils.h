#ifndef UTILS_H
#define UTILS_H



#include <cmath>

#include <Eigen/Dense>


#include <tf/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nav_msgs/Path.h>

namespace PRM{

    
    //struct Node2d; 
    //struct Node3d;

    namespace Utils{

        static inline void mapToWorld(const int mx, const int my, float& wx, float& wy) 
        {   

            float origin_x_ = Constants::MapMetaData::origin_x_; 
            float origin_y_ = Constants::MapMetaData::origin_y_; 
            
            float resolution_ = Constants::MapMetaData::cell_size_;

            wx = origin_x_ + (mx + 0.5) * resolution_;
            wy = origin_y_ + (my + 0.5) * resolution_;
        
        };
        static inline float norm(const float x_, const float y_)
        {

            return sqrt(pow(x_, 2) + pow(y_, 2));
        }

        static inline float fmod_(const float y_, const float x_)
        {

            float r_  = std::fmod(y_, x_);

            return (r_ >= 0.f ? r_  : r_ + x_);
        
        }

        static inline float getR(const float x_, const float y_)
        {
            
            if(std::fabs(y_) < 0.001f)
            {

                // /ROS_ERROR("y_ == 0.f  ==> Something is wrong!");
                return -1.f;
            }

            float R_ = sqrt(pow((x_ * x_ + 2 * Constants::Vehicle::a2_ * x_ + y_ * y_)/ (2.f * y_), 2) + pow(Constants::Vehicle::a2_, 2));            
            return R_;
        }

        static inline float getR(const float delta_)
        {   
            if(std::fabs(delta_) < 0.01f)
            {
                return -1.f;
            }
            
            float R_ = sqrt(pow(Constants::Vehicle::a2_, 2) + pow(PRM::Constants::Vehicle::l_ * (1/std::tan(delta_)), 2));
            return R_;
        
        }

        
        
        //x_dash_, y_dash_ ==> config_pose in robot frame
        static inline float signDelta(const float x_dash_, const float y_dash_)
        {
            if(std::fabs(y_dash_) < 0.001f)
            {
                return 0.f;
            }

            if(x_dash_ > 0)
            {
                return (y_dash_ >= 0 ? 1.f : -1.f) ;
                
            }
            else 
            {

                if(x_dash_ < - 2 * Constants::Vehicle::a2_ )
                {

                    return (y_dash_ >= 0 ? 1.f : -1.f);           

                }
                else
                {

                    return (y_dash_ < 0 ? 1.f : -1.f);

                }

            }

            return 0.f;

        }

        static inline float getThetaC(const float x_dash_, const float y_dash_, const float steering_angle_)
        {

            if(std::fabs(y_dash_) < 0.001) {return 0.f ;}

            const float R_ = getR(x_dash_, y_dash_);

            //float theta_c_ = std::atan((x_dash_ + Constants::Vehicle::a2_) / (sqrt(R_ * R_  - pow(x_dash_ + Constants::Vehicle::a2_, 2))));
            float theta_c_ = std::atan2((x_dash_ + Constants::Vehicle::a2_) , (sqrt(R_ * R_  - pow(x_dash_ + Constants::Vehicle::a2_, 2))));
            //float theta_c_ = std::atan2((sqrt(R_ * R_  - pow(x_dash_ + Constants::Vehicle::a2_, 2))), (x_dash_ + Constants::Vehicle::a2_) );
                    
            //float theta_c_ = std::atan2((x_ + Constants::Vehicle::a2_) , (sqrt(R_ * R_  - pow(x_ + Constants::Vehicle::a2_, 2))));

           // ROS_INFO("theta_c: %f", theta_c_);

            //if(y_dash_ < 0) theta_c_ = -theta_c_; 

            if(steering_angle_ < 0) 
            {
                theta_c_ = -theta_c_;
            }
            
            if(theta_c_ < 0.f) 
            {
                theta_c_ += 2 * M_PI;
            }
            
            return theta_c_;
            
        }

        //steering radius to reach (x_,y_) in the ROBOT FRAME
            

        static inline geometry_msgs::Quaternion getQuatFromYaw(const float yaw_)
        {
            tf2::Quaternion quaternion;
            quaternion.setRPY(0.0, 0.0, yaw_);

            // Convert the quaternion to a geometry_msgs::Quaternion
            geometry_msgs::Quaternion msg_quaternion;
            tf2::convert(quaternion, msg_quaternion);

            return msg_quaternion;
        }

        

        static inline Eigen::Matrix3f getHomogeneousTransformationMatrix(const Eigen::Vector2f &translation, const float &theta) 
        {
            Eigen::Rotation2Df rotation(theta);

            //std::cout << "Transformation matrix:\n" << rotation.matrix() << std::endl;

            // Define the translation vector
            //Eigen::Vector2d translation(1.0, 2.0); // Example translation vector [1, 2]

            // Create a homogeneous transformation matrix
            Eigen::Affine2f transformation;
            transformation.setIdentity(); // Initialize to identity matrix
            transformation.linear() = rotation.toRotationMatrix();
            transformation.translation() = translation;

            // Print the transformation matrix
            //std::cout << "Transformation matrix:\n" << transformation.matrix() << std::endl;

            return transformation.matrix();

        }

        static inline void displayPlannerParams()
        {

            ROS_INFO("delta_max_: %f", Constants::Vehicle::delta_max_);
            ROS_INFO("a2_: %f", Constants::Vehicle::a2_);
            ROS_INFO("l_: %f", Constants::Vehicle::l_);
            ROS_INFO("r_min_: %f", Constants::Vehicle::R_MIN_);
            ROS_INFO("max_res_: %f", Constants::Planner::max_res_);

        }

        


    };


};


#endif