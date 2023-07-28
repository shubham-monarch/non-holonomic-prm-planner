#ifndef HELPERS_H
#define HELPERS_H

#include <non-holonomic-prm-planner/ds.h>
#include <non-holonomic-prm-planner/steering_curve.h>

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

        static inline Vec3f getNode3dkey(const Node3d &node_)
        {
            return Vec3f{node_.x_, node_.y_, 1.f * node_.theta_idx_};
        }

        static inline Vec3f getNode3dkey(const geometry_msgs::Pose &pose_)
        {   
            const int theta_idx_ = tf::getYaw(pose_.orientation) / Constants::Planner::theta_sep_;
            return Vec3f{pose_.position.x ,  pose_.position.y, 1.f * theta_idx_};
        }

        static inline nav_msgs::Path generateROSPath(const std::vector<Node3d>&path_)
        {

            ROS_INFO("Inside generateROSPath function!");


            int sz_ = (int)path_.size(); 

            //nav_msgs::Path path_;
            nav_msgs::Path ros_path_;
            ros_path_.header.frame_id = "map"; 
            ros_path_.header.stamp = ros::Time::now();

            geometry_msgs::PoseArray final_path_; 
            final_path_.header.frame_id = "map" ; 
            final_path_.header.stamp = ros::Time::now();

            


            for(int i =0 ; i < sz_ - 1; i++)
            {   
                //std::cout << "i: " << i << std::endl;
                geometry_msgs::Pose a_; 
                a_.position.x = path_[i].x_; 
                a_.position.y = path_[i].y_;
                a_.orientation = Utils::getQuatFromYaw(path_[i].theta_);
                //a_.orientation = Utils::getQuatFromYaw(0.f);
                    

                geometry_msgs::Pose b_; 
                b_.position.x = path_[i + 1].x_; 
                b_.position.y = path_[i+ 1].y_;
                b_.orientation = Utils::getQuatFromYaw(path_[i + 1].theta_);

                //generateSteeringCurveFamily(a_, "final_family_" + std::to_string(i));
            // SteeringCurve::generateSteeringCurveFamily(path_[i], "aa_" + std::to_string(i));
                
                const std::vector<geometry_msgs::PoseStamped> poses_ = SteeringCurve::generateSteeringCurveTrimmed(a_, b_);

                for(const auto t: poses_)
                {
                    auto pose_ = t.pose; 
                    final_path_.poses.push_back(pose_);
                }

                //ros_path_.poses.push_back(sc_);

                ros_path_.poses.insert(ros_path_.poses.end(), std::make_move_iterator(poses_.begin()), std::make_move_iterator(poses_.end()));
                //final_path_.poses.insert(final_path_.poses.end(), std::make_move_iterator(poses_.begin()), std::make_move_iterator(poses_.end()));
            
            }

            //visualize_.publishT<nav_msgs::Path>("path", ros_path_);
            //visualize_.publishT<geometry_msgs::PoseArray>("final_path", final_path_);


            //ROS_INFO("Visualiztion finished!");
            return ros_path_;
        }


    };   
    
    


};



#endif