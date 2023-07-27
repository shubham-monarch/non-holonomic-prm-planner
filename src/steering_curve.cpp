#include <non-holonomic-prm-planner/steering_curve.h>
#include <non-holonomic-prm-planner/ds.h>

#include <Eigen/Dense>
#include <eigen3/Eigen/Core>

geometry_msgs::PoseArray PRM::SteeringCurve::generateSteeringCurve( geometry_msgs::Pose rp_, const float R_, \
                                                                const bool trim_, \
                                                                const float del_sign_, const float x_dash_)
{
    
    //ROS_ERROR("generateSteeringCurve called with R_: %f", R_);
   // return geometry_msgs::PoseArray();

    //delta_ = 0.577;
    ////delta_ = 0.1;
    //ROS_INFO("Inside generateSteeringCurve function!");
    //ROS_INFO("R_: %f", R_); 
    //homogenous co-ordinatesS
    //[cosθ     sinθ    x
    // -sinθ    cosθ    y 
    //  0       0       1]

    Eigen::Matrix3f P_oa_;   //robot pose in world frame 
   // Eigen::Matrix3f P_ab;   //point pose in robot frame
   // Eigen::Matrix3f P_ob_;   //point pose in world frame
    
    //double theta = tf::getYaw(robot_pose_.orientation);

    
    P_oa_ = Utils::getHomogeneousTransformationMatrix(Eigen::Vector2f(rp_.position.x , rp_.position.y), tf::getYaw(rp_.orientation));

    // /ROS_INFO_STREAM("P_oa: \n", P_oa);

   // std::cout << "P_oa: " << std::endl;
    //std::cout << P_oa_ << std::endl;

    /*float R_;
    if(std::fabs(delta_) > 0.001) 
    {
        R_ = Utils::getR(delta_);

    } 
    ROS_WARN("R_: %f", R_);
    */

    std::vector<Eigen::Vector2f> V_ab_pos_, V_ab_neg_, V_ab_zero_; // point pose in robot frame
    V_ab_pos_.reserve(1000);
    V_ab_neg_.reserve(1000);
    V_ab_zero_.reserve(1000);
    
    
    //assuming δ > 0
    
  //  ROS_WARN("Case 1 ===>");
    // /ROS_WARN("R_: %f" , R_);
    
    if(R_ > 0.f)
    {
        
        //for (float x_ = 0.f ;  ; x_ += 0.1f)
        
        for (float x_ = 0.f ;   ; x_ += Constants::Planner::dis_sep_)
        {
            
            if(trim_ && x_dash_ < 0)
            {
                break;
            }

            if(trim_ && (x_ > x_dash_))
            {
                break;
            }


            float y_ ; 
            //if(delta_ > 0.f){


            y_ = -sqrt(pow(R_, 2) - pow(x_ + Constants::Vehicle::a2_,2)) + sqrt(pow(R_,2 ) - pow(Constants::Vehicle::a2_, 2));
            //ROS_INFO("y_: %f", y_);

            if(std::isnan(y_)) {
            
                //ROS_WARN("Last x_ value: %f", x_);
                break;
            }
        
            V_ab_pos_.emplace_back(x_,y_);
            //V_ab_zero_.emplace_back(x_, 0);

            //ROS_INFO("(x,y) ==> (%f,%f)", x_, y_);
        
            y_ = sqrt(pow(R_, 2) - pow(x_ + Constants::Vehicle::a2_,2)) - sqrt(pow(R_,2 ) - pow(Constants::Vehicle::a2_, 2));
            //ROS_INFO("y_: %f", y_);
            
            if(std::isnan(y_)) {
            
               // ROS_WARN("Last x_ value: S%f", x_);
                break;
            }
        
            V_ab_neg_.emplace_back(x_,y_);
            //V_ab_zero_.emplace_back(x_, 0);
        }

       // ROS_WARN("Case 2 ===>");

        for (float x_ = 0.f ;  ; x_ -= Constants::Planner::dis_sep_)
        {
            
            if(trim_ && x_dash_ > 0)
            {
                break;
            }

            if(trim_ && (x_ < x_dash_))
            {
                break;
            }

            float y_ ; 
            //if(delta_ > 0.f){

            
            y_ = -sqrt(pow(R_, 2) - pow(x_ + Constants::Vehicle::a2_,2)) + sqrt(pow(R_,2 ) - pow(Constants::Vehicle::a2_, 2));
            
            if(std::isnan(y_)) {
            
               // ROS_WARN("Last x_ value: %f", x_);
                break;
            }
        
            V_ab_pos_.emplace_back(x_,y_);
            //V_ab_zero_.emplace_back(x_, 0);
            
            //ROS_INFO("(x,y) ==> (%f,%f)", x_, y_);
        

            y_ = sqrt(pow(R_, 2) - pow(x_ + Constants::Vehicle::a2_,2)) - sqrt(pow(R_,2 ) - pow(Constants::Vehicle::a2_, 2));
            
            if(std::isnan(y_)) {
            
               // ROS_WARN("Last x_ value: %f", x_);
                break;
            }
        
            V_ab_neg_.emplace_back(x_,y_);
        // V_ab_zero_.emplace_back(x_, 0);
            
        }
    }

    else
    {

        for(float x_ =0 ; x_ < Constants::Planner::max_res_; x_+= Constants::Planner::dis_sep_)
        {
            if(x_ > x_dash_)
            {
                break;
            }
            V_ab_zero_.emplace_back(x_, 0);
        }

        for(float x_ =0 ; x_ > -Constants::Planner::max_res_; x_-= Constants::Planner::dis_sep_)
        {
            
            if(x_ < x_dash_)
            {
                break;
            }

            V_ab_zero_.emplace_back(x_, 0);
        }

    }
   


    //for(auto t: V_ab_) std::cout << t(0) << " " << t(1) << std::endl;

    ///return;

    const int sz_ = ((int)V_ab_pos_.size() +  (int)V_ab_neg_.size() + (int)V_ab_zero_.size());

    //ROS_INFO("sz_: %d", sz_);

    //std::vector<Eigen::Matrix3f> V_ob_;

    std::vector<geometry_msgs::Pose> poses_ob_;

    poses_ob_.reserve(sz_ + 100000);

    if(trim_ && del_sign_ > 0.f)
    {
        for(const auto &t: V_ab_pos_)
        {    
            //ROS_INFO("x: %f y: %f ", t[0], t[1]);
            const float yaw_ = Utils::getThetaC(t[0], t[1], 1.f);
            //const float yaw_ = 0.f;
            //ROS_DEBUG("yaw_: %f", yaw_); 
            const Mat3f &P_ab_ = Utils::getHomogeneousTransformationMatrix(t, yaw_);
            //const Mat3f &P_ab_ = Utils::getHomogeneousTransformationMatrix(t, 0.0);

            //const Mat3f &P_ob_ = P_ab_ * P_oa_;
            const Mat3f &P_ob_ = P_oa_ * P_ab_;
            
            geometry_msgs::Pose pose_ob_;  //pose of b in world frame
            pose_ob_.position.x = P_ob_(0,2); 
            pose_ob_.position.y = P_ob_(1,2);
            pose_ob_.orientation = Utils::getQuatFromYaw(std::atan2(P_ob_(1,0), P_ob_(0,0)));
            //pose_ob_.position.x = t(0) + rp_.position.x;  
            //pose_ob_.position.y = t(1) + rp_.position.y;

            poses_ob_.push_back(pose_ob_);    

        }
    }

    if(trim_ && del_sign_ < 0.f)
    {
        for(const auto &t: V_ab_neg_)
        {    
            //ROS_INFO("x: %f y: %f ", t[0], t[1]);
            const float yaw_ = Utils::getThetaC(t[0], t[1], -1.f);
            //const float yaw_ = 0.f;
            //ROS_DEBUG("yaw_: %f", yaw_); 
            const Mat3f &P_ab_ = Utils::getHomogeneousTransformationMatrix(t, yaw_);
            //const Mat3f &P_ab_ = Utils::getHomogeneousTransformationMatrix(t, 0.0);

            //const Mat3f &P_ob_ = P_ab_ * P_oa_;
            const Mat3f &P_ob_ = P_oa_ * P_ab_;
            
            geometry_msgs::Pose pose_ob_;  //pose of b in world frame
            pose_ob_.position.x = P_ob_(0,2); 
            pose_ob_.position.y = P_ob_(1,2);
            pose_ob_.orientation = Utils::getQuatFromYaw(std::atan2(P_ob_(1,0), P_ob_(0,0)));
            //pose_ob_.position.x = t(0) + rp_.position.x;  
            //pose_ob_.position.y = t(1) + rp_.position.y;

            poses_ob_.push_back(pose_ob_);    

        }   
    }

    if(trim_ && std::fabs(del_sign_) < 0.01)
    {
        for(const auto &t: V_ab_zero_)
        {
            ///ROS_INFO("x: %f y: %f ", t[0], t[1]);
            const float yaw_ = Utils::getThetaC(t[0], t[1], 0.f);
            //const float yaw_ = 0.f;
            //ROS_DEBUG("yaw_: %f", yaw_); 
            const Mat3f &P_ab_ = Utils::getHomogeneousTransformationMatrix(t, yaw_);
            //const Mat3f &P_ab_ = Utils::getHomogeneousTransformationMatrix(t, 0.0);

            //const Mat3f &P_ob_ = P_ab_ * P_oa_;
            const Mat3f &P_ob_ = P_oa_ * P_ab_;
            
            geometry_msgs::Pose pose_ob_;  //pose of b in world frame
            pose_ob_.position.x = P_ob_(0,2); 
            pose_ob_.position.y = P_ob_(1,2);
            pose_ob_.orientation = Utils::getQuatFromYaw(std::atan2(P_ob_(1,0), P_ob_(0,0)));
            //pose_ob_.position.x = t(0) + rp_.position.x;  
            //pose_ob_.position.y = t(1) + rp_.position.y;

            poses_ob_.push_back(pose_ob_);    

        }
    }
    
    //ROS_INFO("poses_ob_.size(): %d" , poses_ob_.size());

    geometry_msgs::PoseArray pose_array_ob_; 
    pose_array_ob_.header.frame_id = "map"; 
    pose_array_ob_.header.stamp = ros::Time::now();

    pose_array_ob_.poses = std::move(poses_ob_);
    
    return pose_array_ob_;

}