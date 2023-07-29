#include <non-holonomic-prm-planner/steering_curve.h>
#include <non-holonomic-prm-planner/ds.h>
#include <non-holonomic-prm-planner/roadmap.h>

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



//generate steering curves from -delta to delta for a particular robot pose
geometry_msgs::PoseArray PRM::SteeringCurve::generateSteeringCurveFamily(geometry_msgs::Pose rp_, std::string topic_)
{

    float del_ = 0.f;

   // ROS_INFO("max_steering_angle => %f", Constants::Vehicle::delta_max_);

    int cnt_ =0 ; 

    geometry_msgs::PoseArray family_;
    family_.header.frame_id = "map"; 
    family_.header.stamp = ros::Time::now();

    while(ros::ok() && del_ <= Constants::Vehicle::delta_max_)
    {   
        //ROS_INFO("del_: %f", del_);   
        const float R_ = Utils::getR(del_);

        //ROS_WARN("del_: %f R_: %f" , del_, R_);

        geometry_msgs::PoseArray arr_ = SteeringCurve::generateSteeringCurve(rp_, R_);
        
        family_.poses.insert(family_.poses.end(), \
                            std::make_move_iterator(arr_.poses.begin()),
                            std::make_move_iterator(arr_.poses.end()));

        
        del_ += Constants::Planner::theta_sep_;
        cnt_++;
        
    }

    return family_;


}

geometry_msgs::PoseArray PRM::SteeringCurve::generateSteeringCurveFamily(const Node3d &node_, std::string topic_)
{

    geometry_msgs::Pose rp_; 
    rp_.position.x = node_.x_; 
    rp_.position.y = node_.y_ ;
    rp_.orientation = Utils::getQuatFromYaw(node_.theta_); 

    return generateSteeringCurveFamily(rp_, topic_);

}

//returns the portion of sterring curve between or_ (i.e. position of r w.r.t. origin) and oc_ (position of c w.r.t origin)
std::vector<geometry_msgs::PoseStamped> PRM::SteeringCurve::generateSteeringCurveTrimmed(   const geometry_msgs::Pose &or_, \
                                                                                            const geometry_msgs::Pose &oc_)
{   
   

    Mat3f P_or_; //robot pose in origin frame
    Mat3f P_oc_; // config pose in origin frame
    //Mat3f P_rc_; //config pose in robot frame

    Vec2f V_or_{or_.position.x, or_.position.y};
    float theta_or_ = tf::getYaw(or_.orientation);

    Vec2f V_oc_{oc_.position.x, oc_.position.y};
    float theta_oc_ = tf::getYaw(oc_.orientation);

    P_or_ = (Utils::getHomogeneousTransformationMatrix(V_or_, theta_or_));
    P_oc_ = (Utils::getHomogeneousTransformationMatrix(V_oc_, theta_oc_));
    
    std::ostringstream oss_; 
    
    const Mat3f &P_ro_ = P_or_.inverse();

    const Mat3f &P_rc_ = P_ro_ * P_oc_;

    
    //config co-ordinates in robot frame
    const float x_dash_ =  P_rc_(0,2); 
    const float y_dash_ = P_rc_(1,2); 

    const float R_ = Utils::getR(x_dash_, y_dash_); 

    const float del_sign_ = Utils::signDelta(x_dash_, y_dash_);

    geometry_msgs::PoseArray sc_poses_ = SteeringCurve::generateSteeringCurve(or_,  R_, true, del_sign_,  x_dash_);

    std::vector<geometry_msgs::PoseStamped> poses_; 

    for(const auto &t_ : sc_poses_.poses)
    {
        geometry_msgs::PoseStamped pose_; 
        pose_.header.frame_id = "map"; 
        pose_.header.stamp  = ros::Time::now(); 

        pose_.pose = t_; 

        poses_.push_back(pose_);
    }

    return poses_;

}

bool PRM::SteeringCurve::connectConfigurationToRobot(   const Node3d &rp_, const Node3d &cp_, \
                                                        const std::string or_topic_, const std::string oc_topic_, \
                                                        const std::string sc_topic_) 
{

    geometry_msgs::Pose or_;
    or_.position.x = rp_.x_;
    or_.position.y = rp_.y_;
    or_.orientation = Utils::getQuatFromYaw(1.f * rp_.theta_idx_ * Constants::Planner::theta_sep_); 

    geometry_msgs::Pose oc_; 
    oc_.position.x = cp_.x_; 
    oc_.position.y = cp_.y_;
    oc_.orientation = Utils::getQuatFromYaw(1.f * cp_.theta_idx_ * Constants::Planner::theta_sep_);

    
    //ROS_INFO("or_: (%f,%f,%f)" ,    or_.position.x, or_.position.y, tf::getYaw(or_.orientation));
    //ROS_INFO("oc_: (%f,%f,%f)" ,    oc_.position.x, oc_.position.y, tf::getYaw(oc_.orientation));
    
    //ROS_INFO("or_topic_: %s", or_topic_.c_str());
    //ROS_INFO("oc_topic_: %s", oc_topic_.c_str());
    //ROS_INFO("sc_topic_: %s", sc_topic_.c_str());

    connectConfigurationToRobot(or_, oc_, or_topic_, oc_topic_, sc_topic_);

    return true;
}


bool PRM::SteeringCurve::connectConfigurationToRobot(   geometry_msgs::Pose or_ , geometry_msgs::Pose oc_, \
                                                        const std::string or_topic_, const std::string oc_topic_, \
                                                        const std::string sc_topic_) 
{

    
    //ROS_INFO("connectConfigToRobot called ==> or_topic_: %s oc_topic: %s sc_topic: %s" , or_topic_.c_str(), oc_topic_.c_str(), sc_topic_.c_str());
    /*or_.position.x = -937.f;
    or_.position.y = -245.f;
    or_.orientation = Utils::getQuatFromYaw(0.f);

    oc_.position.x = -935.f;
    oc_.position.y = -243.f; 
    oc_.orientation = Utils::getQuatFromYaw(0.f);
    */

    Mat3f P_or_; //robot pose in origin frame
    Mat3f P_oc_; // config pose in origin frame
    //Mat3f P_rc_; //config pose in robot frame

    Vec2f V_or_{or_.position.x, or_.position.y};
    float theta_or_ = tf::getYaw(or_.orientation);

    Vec2f V_oc_{oc_.position.x, oc_.position.y};
    float theta_oc_ = tf::getYaw(oc_.orientation);



    P_or_ = (Utils::getHomogeneousTransformationMatrix(V_or_, theta_or_));
    P_oc_ = (Utils::getHomogeneousTransformationMatrix(V_oc_, theta_oc_));
    
    std::ostringstream oss_; 
    
    oss_ << P_or_; 
    //ROS_INFO("P_or_: \n%s", oss_.str().c_str());

    oss_.str("");
    oss_ << P_oc_;
   // ROS_INFO("P_oc_: \n%s", oss_.str().c_str());

    const Mat3f &P_ro_ = P_or_.inverse();

    oss_.str("");
    oss_ << P_ro_;
   // ROS_INFO("P_ro_: \n%s", oss_.str().c_str());


    const Mat3f &P_rc_ = P_ro_ * P_oc_;

    oss_.str("");
    oss_ << P_rc_;
    //ROS_INFO("P_rc_: \n%s", oss_.str().c_str());

    
    //config co-ordinates in robot frame
    const float x_dash_ =  P_rc_(0,2); 
    const float y_dash_ = P_rc_(1,2); 

    const float R_ = Utils::getR(x_dash_, y_dash_); 

    //ROS_WARN("(x_dash, y_dash) => (%f,%f)", x_dash_, y_dash_);

//    return false;
    //ROS_DEBUG("Checking R_ inside connectConfigurationToRobot R_: %f", R_);
  //  ros::Duration(5.0).sleep();

    //geometry_msgs::PoseArray sc_poses_ = generateSteeringCurve(or_, R_);
    geometry_msgs::PoseArray sc_poses_ = SteeringCurve::generateSteeringCurve(or_,  R_);

    visualize_->publishT<geometry_msgs::PoseArray>(sc_topic_, sc_poses_);

    //ROS_INFO("oc_: (%f,%f)", oc_.position.x, oc_.position.y);

    visualize_->drawPoint(or_, or_topic_);
    visualize_->drawPoint(oc_, oc_topic_);
    
    //add yaw
    //visualize_.drawPoint(x_, y_); //
    
    return true; 

}




