#ifndef UTILS_H
#define UTILS_H

#include <non-holonomic-prm-planner/constants.h>

#include <cmath>

#include <Eigen/Dense>


#include <tf/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>




namespace PRM{

    namespace Utils{

        static inline void mapToWorld(const int mx, const int my, float& wx, float& wy) 
        {   

            float origin_x_ = Constants::MapMetaData::origin_x_; 
            float origin_y_ = Constants::MapMetaData::origin_y_; 
            
            float resolution_ = Constants::MapMetaData::cell_size_;

            wx = origin_x_ + (mx + 0.5) * resolution_;
            wy = origin_y_ + (my + 0.5) * resolution_;
        
        };

        static inline float euclidean(const Node2d &a_, const Node2d &b_)
        {

            return sqrt(pow(a_.x_ - b_.x_ , 2) + pow(a_.y_ - b_.y_, 2));

        }

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
            
            if(y_ == 0.f)
            {

                ROS_ERROR("y_ == 0.f  ==> Something is wrong!");
                return -1.f;
            }

            float R_ = sqrt(pow((x_ * x_ + 2 * Constants::Vehicle::a2_ * x_ + y_ * y_)/ (2.f * y_), 2) + pow(Constants::Vehicle::a2_, 2));            
            return R_;
        }


        static inline float getR(const float delta_)
        {
            float R_ = sqrt(pow(Constants::Vehicle::a2_, 2) + pow(PRM::Constants::Vehicle::l_ * (1/std::tan(delta_)), 2));
            return R_;
        
        }

        static inline float getThetaC(const float x_, const float y_)
        {

            if(y_ == 0.f) {return 0.f ;}

            const float R_ = getR(x_, y_);

            float theta_c_ = std::atan((x_ + Constants::Vehicle::a2_) / (sqrt(R_ * R_  - pow(x_ + Constants::Vehicle::a2_, 2))));

            if(y_ < 0) theta_c_ = -theta_c_; 

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