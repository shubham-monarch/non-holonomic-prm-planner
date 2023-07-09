#include <non-holonomic-prm-planner/simple_prm.h>
#include <non-holonomic-prm-planner/constants.h>
#include <non-holonomic-prm-planner/flags.h>
#include <non-holonomic-prm-planner/utils.h>

#include <random>
#include <Eigen/Dense>
#include <eigen3/Eigen/Core>


#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>

#include <cmath>

typedef Eigen::Matrix3f Mat3f;
typedef Eigen::Vector2f Vec2f;


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

    nodes2d_.clear();

    ROS_INFO("r_min_: %f", Constants::Vehicle::r_min_); 
    ROS_INFO("max_res_: %f", Constants::Vehicle::max_res_);

    

}



bool PRM::SimplePRM::isObstacleFree(const Node2d &node_) const
{

    return true; 
}


bool PRM::SimplePRM::buildKDtree()
{

    


}

bool PRM::SimplePRM::generateRoadMap()
{
   // ROS_INFO("Inside PRM::plan()");

    generateSamplePoints();
    //buildKDtree();
    //geometry_msgs::Pose pose_;
    //generateSteeringCurve(geometry_msgs::Pose(), 0.0);
    //generateSteeringCurveFamily(pose_);

    //generateEdges();
    //generatePath();

    //connectConfigurationToRobot(pose_, pose_);

    //generateKDTree();


    
    return true; 
}

void PRM::SimplePRM::setMapCb(nav_msgs::OccupancyGrid::ConstPtr map_)
{

    Constants::MapMetaData::origin_x_ = map_->info.origin.position.x;
    Constants::MapMetaData::origin_y_ = map_->info.origin.position.y;
    Constants::MapMetaData::cell_size_ = map_->info.resolution;
    Constants::MapMetaData::height_ = map_->info.height;
    Constants::MapMetaData::width_ = map_->info.width;
    Constants::MapMetaData::res_ = map_->info.resolution;
    
    grid_ = map_;

    ROS_DEBUG("MAP PARAMS ===>");
    ROS_DEBUG("dimensions: (%d, %d)", map_->info.height, map_->info.width);
    ROS_DEBUG("resolution: %f", map_->info.resolution);
    ROS_DEBUG("origin: (%f, %f)", map_->info.origin.position.x, map_->info.origin.position.y);

    map_set_ = true; 
    
    
}


bool PRM::SimplePRM::connectConfigurationToRobot(geometry_msgs::Pose or_ , geometry_msgs::Pose oc_) 
{

    or_.position.x = -937.f;
    or_.position.y = -245.f;
    or_.orientation = Utils::getQuatFromYaw(0.f);

    oc_.position.x = -935.f;
    oc_.position.y = -243.f; 
    oc_.orientation = Utils::getQuatFromYaw(0.f);

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
    ROS_INFO("P_or_: \n%s", oss_.str().c_str());

    oss_.str("");
    oss_ << P_oc_;
    ROS_INFO("P_oc_: \n%s", oss_.str().c_str());

    const Mat3f &P_ro_ = P_or_.inverse();

    oss_.str("");
    oss_ << P_ro_;
    ROS_INFO("P_ro_: \n%s", oss_.str().c_str());


    const Mat3f &P_rc_ = P_ro_ * P_oc_;

    oss_.str("");
    oss_ << P_rc_;
    ROS_INFO("P_rc_: \n%s", oss_.str().c_str());

    
    //config co-ordinates in robot frame
    const float x_ =  P_rc_(0,2); 
    const float y_ = P_rc_(1,2); 

    const float R_ = Utils::getRfromConfigPose(x_, y_); 

    generateSteeringCurve(or_, oc_, R_);

    return true; 

}

bool PRM::SimplePRM::generateSamplePoints(){

    ROS_INFO("Inside simplePRM::samplePoints!");

    //grid width and height
    const int w_ = Constants::MapMetaData::width_; 
    const int h_ =  Constants::MapMetaData::height_;

    ROS_INFO("map_dimension: (%f,%f)", h_, w_);

    const float xi_ = Constants::MapMetaData::origin_x_;
    const float xf_ = Constants::MapMetaData::origin_x_ + (w_ + 0.5) * Constants::MapMetaData::res_;
    
    const float yi_ = Constants::MapMetaData::origin_y_;
    const float yf_ = Constants::MapMetaData::origin_y_ + (h_ + 0.5) * Constants::MapMetaData::res_;
    
    ROS_INFO("xi_: %f", xi_);
    ROS_INFO("xf_: %f", xf_);
    ROS_INFO("yi_: %f", yi_);
    ROS_INFO("yf_: %f", yf_);
    
    std::random_device rd;
    std::mt19937 gen(rd());
    
    // Define the range for x, y, and theta



  
    

    std::uniform_real_distribution<float> dist_x(xi_, xf_);
    std::uniform_real_distribution<float> dist_y(yi_, yf_);
    //std::uniform_real_distribution<float> dist_theta(0.0, 2 * M_PI);
    
    
    if((int)nodes2d_.size() > 0) {

        ROS_ERROR("nodes_.size() > 0!"); 
        return false;

    }
     
    nodes2d_.reserve(N_);

    while((int)nodes2d_.size()  < N_) {

        float x_ = dist_x(gen);
        float y_ = dist_y(gen);
        
        Node2d node2d_(x_, y_);

        if(isObstacleFree(node2d_)) {

            nodes2d_.push_back(node2d_);

        }

    }
    
    ROS_DEBUG("nodes2d_.size(): %d", nodes2d_.size());
    
    geometry_msgs::PoseArray pose_array_;
    pose_array_.header.frame_id = "map"; 
    pose_array_.header.stamp = ros::Time::now(); 



    for(const auto &t: nodes2d_) {

        geometry_msgs::Pose pose_; 
        pose_.position.x = t.x_;
        pose_.position.y = t.y_; 
        pose_.orientation = Utils::getQuatFromYaw(0.f);

        pose_array_.poses.push_back(pose_);

    }   

   visualize_.visualizeSampledPoints(pose_array_);

    return true; 

}

//generate steering curves from -delta to delta for a particular robot pose
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
    rp_.orientation = Utils::getQuatFromYaw(0.1f);
    
    P_oa_ = Utils::getHomogeneousTransformationMatrix(Eigen::Vector2f(rp_.position.x , rp_.position.y), tf::getYaw(rp_.orientation));

    // /ROS_INFO_STREAM("P_oa: \n", P_oa);

    std::cout << "P_oa: " << std::endl;
    std::cout << P_oa_ << std::endl;

    float R_;
    if(std::fabs(delta_) > 0.001) 
    {
        R_ = Utils::getR(delta_);

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
            
            // /ROS_WARN("delta_ is 0.0f");
            y_ = 0.f;

        }

        if(std::isnan(y_)) {
            
            //ROS_WARN("Last x_ value: %f", x_);
            break;
        }
        
        //Eigen::Vector2f v(x_, y_);

        //ROS_INFO("R: %f" , R_);
        // /ROS_INFO("delta_: %f", delta_);
        //ROS_WARN("(x_,y_) => (%f,%f)", x_, y_);
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
        
        const Mat3f &P_ab_ = Utils::getHomogeneousTransformationMatrix(t, 0.0);

        const Mat3f &P_ob_ = P_ab_ * P_oa_;

        geometry_msgs::Pose pose_ob_;  //pose of b in world frame
        pose_ob_.position.x = P_ob_(0,2); 
        pose_ob_.position.y = P_ob_(1,2);

        //pose_ob_.position.x = t(0) + rp_.position.x;  
        //pose_ob_.position.y = t(1) + rp_.position.y;

        poses_ob_.push_back(pose_ob_);    

    }

   // ROS_INFO("poses_ob_.size(): %d", poses_ob_.size());

    geometry_msgs::PoseArray pose_array_ob_; 
    pose_array_ob_.header.frame_id = "map"; 
    pose_array_ob_.header.stamp = ros::Time::now();
    pose_array_ob_.poses = std::move(poses_ob_);

    if(FLAGS::STEERING_CURVE_VIS)
    {

        visualize_.visualizeSteeringCurve(pose_array_ob_);
    
    }
   
    steering_curve_family_poses_.insert(steering_curve_family_poses_.end(), \
                                        std::make_move_iterator(pose_array_ob_.poses.begin()),   \
                                        std::make_move_iterator(pose_array_ob_.poses.end()));
    
    
    return true;

}   

//generate steering curve points for a particular delta with a config pose
bool PRM::SimplePRM::generateSteeringCurve(geometry_msgs::Pose rp_,  geometry_msgs::Pose &cp_, float R_)
{
    
    //delta_ = 0.577;
    ////delta_ = 0.1;

    ROS_INFO("Inside generateSteeringCurve function!");
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

    std::cout << "P_oa: " << std::endl;
    std::cout << P_oa_ << std::endl;

    /*float R_;
    if(std::fabs(delta_) > 0.001) 
    {
        R_ = Utils::getR(delta_);

    } 
    ROS_WARN("R_: %f", R_);
    */

    std::vector<Eigen::Vector2f> V_ab_;
    V_ab_.reserve(1000);
    //assuming δ > 0
    float y_;  
    for (float x_ = 0.f ;  ; x_ += 0.1f)
    {
        
        float y_ ; 
        //if(delta_ > 0.f){


            y_ = -sqrt(pow(R_, 2) - pow(x_ + Constants::Vehicle::a2_,2)) + sqrt(pow(R_,2 ) - pow(Constants::Vehicle::a2_, 2));
            
            if(std::isnan(y_)) {
            
                ROS_WARN("Last x_ value: %f", x_);
                break;
            }
        
            V_ab_.emplace_back(x_,y_);


            y_ = sqrt(pow(R_, 2) - pow(x_ + Constants::Vehicle::a2_,2)) - sqrt(pow(R_,2 ) - pow(Constants::Vehicle::a2_, 2));
            
            if(std::isnan(y_)) {
            
                ROS_WARN("Last x_ value: %f", x_);
                break;
            }
        
            V_ab_.emplace_back(x_,y_);


        //Eigen::Vector2f v(x_, y_);

        //ROS_INFO("R: %f" , R_);
        //ROS_INFO("delta_: %f", delta_);
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
        
        const Mat3f &P_ab_ = Utils::getHomogeneousTransformationMatrix(t, 0.0);

        const Mat3f &P_ob_ = P_ab_ * P_oa_;

        geometry_msgs::Pose pose_ob_;  //pose of b in world frame
        pose_ob_.position.x = P_ob_(0,2); 
        pose_ob_.position.y = P_ob_(1,2);

        //pose_ob_.position.x = t(0) + rp_.position.x;  
        //pose_ob_.position.y = t(1) + rp_.position.y;

        poses_ob_.push_back(pose_ob_);    

    }

   // ROS_INFO("poses_ob_.size(): %d", poses_ob_.size());

    geometry_msgs::PoseArray pose_array_ob_; 
    pose_array_ob_.header.frame_id = "map"; 
    pose_array_ob_.header.stamp = ros::Time::now();
    pose_array_ob_.poses = std::move(poses_ob_);

    if(FLAGS::STEERING_CURVE_VIS)
    {

        visualize_.visualizeSteeringCurve(pose_array_ob_);
    
    }
   
    steering_curve_family_poses_.insert(steering_curve_family_poses_.end(), \
                                        std::make_move_iterator(pose_array_ob_.poses.begin()),   \
                                        std::make_move_iterator(pose_array_ob_.poses.end()));
    

    geometry_msgs::PoseStamped cp_stamped_;
    cp_stamped_.header.frame_id = "map";
    cp_stamped_.header.stamp = ros::Time::now();
    cp_stamped_.pose = cp_;


    visualize_.visualizePointPose(cp_stamped_);
    return true;




}



