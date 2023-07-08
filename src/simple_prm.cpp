#include <non-holonomic-prm-planner/simple_prm.h>
#include <non-holonomic-prm-planner/constants.h>

#include <random>
#include <Eigen/Dense>

#include <tf/transform_datatypes.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <geometry_msgs/PoseArray.h>

#include <cmath>

typedef Eigen::Matrix3f Mat3f;

PRM::SimplePRM::SimplePRM()
{   

    ROS_WARN("SimplePRM constructor called");

    
}



void PRM::SimplePRM::initialize()
{

    ros::Rate r_(10.0);
    map_sub_ = nh_.subscribe(Constants::map_topic, 1, &SimplePRM::setMapCb, this);
    
    while(ros::ok() && !map_set_){

        ROS_DEBUG("Waiting for map!!");
        ros::spinOnce(); 
    
        r_.sleep();
    }

    ROS_WARN("map_set_ is true!");

    nodes_.clear();

    ROS_INFO("r_min_: %f", Constants::Vehicle::r_min_); 
    ROS_INFO("max_res_: %f", Constants::Vehicle::max_res_);
   

    

}


bool PRM::SimplePRM::isObstacleFree(const Node &node_) const
{

    return true; 
}


bool PRM::SimplePRM::plan(){

    ROS_INFO("Inside PRM::plan()");

    //generateSamplePoints();
    geometry_msgs::Pose pose_;
    //generateSteeringCurve(geometry_msgs::Pose(), 0.0);
    generateSteeringCurveFamily(pose_);

    //generateEdges();
    //generatePath();

    return true; 
}

void PRM::SimplePRM::setMapCb(nav_msgs::OccupancyGrid::ConstPtr map_)
{

    Constants::MapMetaData::origin_x_ = map_->info.origin.position.x;
    Constants::MapMetaData::origin_y_ = map_->info.origin.position.y;
    Constants::MapMetaData::cell_size_ = map_->info.resolution;
    Constants::MapMetaData::height_ = map_->info.height;
    Constants::MapMetaData::width_ = map_->info.width;
    
    grid_ = map_;

    ROS_DEBUG("MAP PARAMS ===>");
    ROS_DEBUG("dimensions: (%d, %d)", map_->info.height, map_->info.width);
    ROS_DEBUG("resolution: %f", map_->info.resolution);
    ROS_DEBUG("origin: (%f, 1%f)", map_->info.origin.position.x, map_->info.origin.position.y);

    map_set_ = true; 
    
    
}


bool PRM::SimplePRM::generateSamplePoints(){

    ROS_INFO("Inside simplePRM::samplePoints!");

    const int width_ = grid_->info.width; 
    const int height_ = grid_->info.height;

    ROS_INFO("grid_: (%f * %f)", height_, width_);

    std::random_device rd;
    std::mt19937 gen(rd());
    
    // Define the range for x, y, and theta
    std::uniform_int_distribution<int> dist_x(0, width_);
    std::uniform_int_distribution<int> dist_y(0, height_);
    std::uniform_real_distribution<float> dist_theta(0.0, 2 * M_PI);
    
    
    if((int)nodes_.size() > 0) {

        ROS_ERROR("nodes_.size() > 0!"); 
        return false;

    }
     
    nodes_.reserve(N_);

    while((int)nodes_.size()  < N_) {

        int x_ = dist_x(gen);
        int y_ = dist_y(gen);
        float theta_ = dist_theta(gen);
        
        Node node_(x_, y_, theta_);

        if(isObstacleFree(node_)) {

            nodes_.emplace_back(node_);

        }

    }
    
    ROS_DEBUG("nodes_.size(): %d", nodes_.size());
    
    
   visualize_.visualizeSampledPoints(nodes_);

    return true; 

}

float getR(const float delta_){

    float R_ = sqrt(pow(PRM::Constants::Vehicle::a2_, 2) + pow(PRM::Constants::Vehicle::l_ * (1/std::tan(delta_)), 2));
    return R_;
}


geometry_msgs::Quaternion getQuatFromYaw(const float yaw_){

    tf2::Quaternion quaternion;
    quaternion.setRPY(0.0, 0.0, yaw_);

    // Convert the quaternion to a geometry_msgs::Quaternion
    geometry_msgs::Quaternion msg_quaternion;
    tf2::convert(quaternion, msg_quaternion);

    return msg_quaternion;
}


Eigen::Matrix3f getHomogeneousTransformationMatrix(const Eigen::Vector2f &translation, const float &theta) {

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

//generate steering curves from -delta to delta
void PRM::SimplePRM::generateSteeringCurveFamily(geometry_msgs::Pose rp_)
{

    ROS_INFO("Inside generateSteeringCurveFamily function!");

    if((int)steering_curve_family_poses_.size() > 0) {

        ROS_WARN("steering_curve_family_poses.size() > 0 ==> Something might be wrong!");
        steering_curve_family_poses_.clear();

    }
    
    ros::Rate r_(10.0); 

    float del_ = -Constants::Vehicle::delta_max_;
    
    ROS_INFO("del_max_: %f", Constants::Vehicle::delta_max_);

    while(ros::ok() && del_ < Constants::Vehicle::delta_max_)
    {   
        ROS_INFO("del_: %f", del_);
        del_ += 0.1;    

        generateSteeringCurve(rp_, del_);

        r_.sleep(); 
    }



    geometry_msgs::PoseArray pose_array_ob_; 
    pose_array_ob_.header.frame_id = "map"; 
    pose_array_ob_.header.stamp = ros::Time::now();
    pose_array_ob_.poses = std::move(steering_curve_family_poses_);

    visualize_.visualizeSteeringCurve(pose_array_ob_);
    
// /    return true;


}


//generate steering curve points for a particular delta
bool PRM::SimplePRM::generateSteeringCurve( geometry_msgs::Pose rp_, float delta_)
{

    if(delta_ > Constants::Vehicle::delta_max_) {
        
        ROS_ERROR("del_max: %f delta_: %f", Constants::Vehicle::delta_max_, delta_);
        ROS_ERROR("***** DELTA_ > DELTA_MAX!! ===> Something is wrong!");
        return false;

    }

    //delta_ = 0.577;
    ////delta_ = 0.1;

    ROS_INFO("Inside generateSteeringCurve function!");
    //homogenous co-ordinates
    //[cosθ     sinθ    x
    // -sinθ    cosθ    y 
    //  0       0       1]

    Eigen::Matrix3f P_oa_;   //robot pose in world frame 
   // Eigen::Matrix3f P_ab;   //point pose in robot frame
   // Eigen::Matrix3f P_ob_;   //point pose in world frame
    
    //double theta = tf::getYaw(robot_pose_.orientation);

    rp_.position.x = -937;
    rp_.position.y = -245;
    rp_.orientation = getQuatFromYaw(0.1f);
    
    P_oa_ = getHomogeneousTransformationMatrix(Eigen::Vector2f(rp_.position.x , rp_.position.y), tf::getYaw(rp_.orientation));

    // /ROS_INFO_STREAM("P_oa: \n", P_oa);

    std::cout << "P_oa: " << std::endl;
    std::cout << P_oa_ << std::endl;

    float R_;
    if(std::fabs(delta_) > 0.001) 
    {
        R_ = getR(delta_);

    } 
    ROS_WARN("R_: %f", R_);

    std::vector<Eigen::Vector2f> V_ab_;
    V_ab_.reserve(1000);
    //assuming δ > 0
    float y_;  
    for (float x_ = 0.f ;  ; x_ += 0.1f)
    {
        
        float y_ ; 
        if(delta_ > 0.f){


            y_ = -sqrt(pow(R_, 2) - pow(x_ + Constants::Vehicle::a2_,2)) + sqrt(pow(R_,2 ) - pow(Constants::Vehicle::a2_, 2));
        
        }

        else if(delta_ < 0.f){

            y_ = sqrt(pow(R_, 2) - pow(x_ + Constants::Vehicle::a2_,2)) - sqrt(pow(R_,2 ) - pow(Constants::Vehicle::a2_, 2));
        

        }

        else if(delta_== 0.f){
            
            ROS_WARN("delta_ is 0.0f");
            y_ = 0.f;

        }

        if(std::isnan(y_)) {
            
            ROS_WARN("Last x_ value: %f", x_);
            break;
        }
        
        //Eigen::Vector2f v(x_, y_);

        //ROS_INFO("R: %f" , R_);
        ROS_INFO("delta_: %f", delta_);
        ROS_WARN("(x_,y_) => (%f,%f)", x_, y_);
        //ROS_INFO("a2_: %f" , Constants::Vehicle::a2_);
        //ROS_INFO("x_ + a2_: %f", x_ + Constants::Vehicle::a2_);
        
        
        V_ab_.emplace_back(x_,y_);

        if(V_ab_.size() > 800) {

            ROS_ERROR("V_ab_.size() ==> %d > 800", (int)V_ab_.size());
            break;
        }


    }

    //for(auto t: V_ab_) std::cout << t(0) << " " << t(1) << std::endl;

    ///return;

    const int sz_ = (int)V_ab_.size(); 

    //std::vector<Eigen::Matrix3f> V_ob_;

    std::vector<geometry_msgs::Pose> poses_ob_;

    poses_ob_.reserve(sz_ + 100);

    for(const auto &t: V_ab_){
        
        const Mat3f &P_ab_ = getHomogeneousTransformationMatrix(t, 0.0);

        const Mat3f &P_ob_ = P_ab_ * P_oa_;

        geometry_msgs::Pose pose_ob_;  //pose of b in world frame
        //pose_ob_.position.x = P_ob_(0,2); 
        //pose_ob_.position.y = P_ob_(1,2);

        pose_ob_.position.x = t(0) + rp_.position.x;  
        pose_ob_.position.y = t(1) + rp_.position.y;

        poses_ob_.push_back(pose_ob_);    

    }

   // ROS_INFO("poses_ob_.size(): %d", poses_ob_.size());

    geometry_msgs::PoseArray pose_array_ob_; 
    pose_array_ob_.header.frame_id = "map"; 
    pose_array_ob_.header.stamp = ros::Time::now();
    pose_array_ob_.poses = std::move(poses_ob_);

    visualize_.visualizeSteeringCurve(pose_array_ob_);
    
    
    steering_curve_family_poses_.insert(steering_curve_family_poses_.end(), std::make_move_iterator(pose_array_ob_.poses.begin()),     std::make_move_iterator(pose_array_ob_.poses.end()));
    
    
    return true;

}   




