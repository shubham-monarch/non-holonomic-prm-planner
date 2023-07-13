#include <non-holonomic-prm-planner/simple_prm.h>
#include <non-holonomic-prm-planner/utils.h>



#include <random>
#include <Eigen/Dense>
#include <eigen3/Eigen/Core>


#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>

#include <cmath>


typedef Eigen::Matrix3f Mat3f;
typedef Eigen::Vector2f Vec2f;
typedef long long int ll;

//forward declaration
float PRM::Constants::MapMetaData::cell_size_; 
float PRM::Constants::MapMetaData::res_; 
float PRM::Constants::MapMetaData::origin_x_; 
float PRM::Constants::MapMetaData::origin_y_;  
int PRM::Constants::MapMetaData::height_; 
int PRM::Constants::MapMetaData::width_; 




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

    ROS_INFO("r_min_: %f", Constants::Vehicle::R_MIN_); 
    ROS_INFO("max_res_: %f", Constants::Planner::max_res_);

}

bool PRM::SimplePRM::isObstacleFree(const Node2d &node_) const
{

    return true; 
}

//void PRM::SimplePRM::generateEdges(const Node2d &a_, const Node)



bool PRM::SimplePRM::buildKDtree()
{

    //kdTree_ = std::make_shared<kdTree::KDTree>(kd_pts_);

    //kdTree::KDTree tree_(kd_pts_);

    ROS_INFO("Inside buildKDTree!");

    if((int)nodes2d_.size() == 0) {

        ROS_ERROR("nodes2d_ is empty!");
        return false;

    }

    kdPoints kd_points_;

    for(const auto &node_: nodes2d_)
    {

        const int mx_ = node_.x_, my_ = node_.y_; 
        float wx_, wy_; 

        Utils::mapToWorld(mx_, my_, wx_, wy_);

        kdPoint kp_ = {wx_, wy_};

        kd_points_.push_back(kp_);

        //ROS_INFO("(mx_, my_): (%d,%d) ==> (wx_, wy_): (%f,%f)", mx_, my_, wx_, wy_);
        //ROS_INFO("(wx_, wy): (%f,%f)");

    }

    kdTree_ = std::make_shared<kdTree::KDTree>(kd_points_);

    return true; 
}




bool PRM::SimplePRM::generateEdges(Node2d c_, Node2d d_)
{
    
    ROS_INFO("generateEdges called!");
    const float ox_ = Constants::MapMetaData::origin_x_; 
    const float oy_ = Constants::MapMetaData::origin_y_;

    //TODO ==> fix for these values
    //const Node2d &a_ = Node2d{ox_  + 30.f, oy_ + 30.f};
    //const Node2d &b_ = Node2d{ox_ + 28.f, oy_ + 30.f};

    const Node2d &a_ = Node2d{ox_  + 31.f, oy_ + 31.f};
    const Node2d &b_ = Node2d{ox_ + 30.f, oy_ + 30.f};

    //a_ = Node2d{ox_  + 30.f, oy_ + 30.f};
    //b_ = Node2d{ox_ + 32.f, oy_ + 32.f};

    const float dis_ = Utils::euclidean(a_, b_) ;

    if(dis_ > Constants::Planner::max_res_)
    {      
        ROS_ERROR("dis_: %f Planner::max_res_: %f", dis_, Constants::Planner::max_res_);
        ROS_ERROR("euclidean distance  > max_res!");
        return false;
    }

    if(dis_ < 0.001f)
    {      
        ROS_ERROR("dis_: %f Planner::max_res_: %f", dis_, Constants::Planner::max_res_);
        ROS_ERROR("euclidean distance  < 0.001!");
        return false;
    }


    const float xa_ = a_.x_, ya_ = a_.y_;
    const float xb_ = b_.x_, yb_ = b_.y_;

    
    const Vec2f V_oa_{xa_, ya_};
    const Vec2f V_ob_{xb_, yb_};
    

    //TODO ==> do intelligent brute-force

    bool found_ = false;

    int cnt_ = 0 ;

    for(float yaw_a_ = 0.0 ; yaw_a_ <= 2 * M_PI ; yaw_a_ += 5 * M_PI / 180.f)
    {
        
        //ROS_INFO("cnt_: %d", cnt_);
        if(cnt_> 10)
        {
            //ROS_INFO("cnt_ > 10 ==> breaking!");
            // /break;
        }
        //homogeneous transformation representing pose of a w.r.t o 
        const Mat3f &P_oa_ = (Utils::getHomogeneousTransformationMatrix(V_oa_, yaw_a_));
        
        const Mat3f &P_ao_ = P_oa_.inverse();
        
        if(found_)
        {
            //break;
        }
            
        

        for (float yaw_b_ = 0.0; yaw_b_  <= 2 * M_PI; yaw_b_ += 5 * M_PI / 180.f)
        {   


            const Mat3f &P_ob_ = (Utils::getHomogeneousTransformationMatrix(V_ob_, yaw_b_));

            const Mat3f &P_ab_ = P_ao_ * P_ob_;  //b in the frame of a
            
            const float x_dash_ = P_ab_(0,2); 
            const float y_dash_ = P_ab_(1,2);

            //ROS_DEBUG("(x_dash, y_dash) ==> (%f, %f)", x_dash_, y_dash_);
           // break;
            const float r_ = Utils::getR(x_dash_, y_dash_);

            //ROS_WARN("r_: %f", r_);
            //ROS_WARN("r_: %f norm(x_dash_, y_dash_): %f", r_, Utils::norm(x_dash_, y_dash_));

            if(r_ > 0.f && r_ < Constants::Vehicle::R_MIN_)
            {   
               // ROS_WARN("yaw_a_: %f === yaw_b_: %f r_: %f", yaw_a_ * 180.f / M_PI, yaw_b_ * 180.f / M_PI, r_);
                continue;
            }

            
            float theta_dash_  = std::atan2(P_ab_(1,0), P_ab_(0,0));

            
            if(theta_dash_ < 0) {theta_dash_ += 2 *  M_PI; }
            
            //const float theta_c_  = Utils::getThetaC(x_dash_, y_dash_);
            //const float theta_c_  = Utils::getThetaC(x_dash_, y_dash_, P_ob_(1,2));
            const float steering_dir_ = Utils::signDelta(x_dash_, y_dash_);
            const float theta_c_  = Utils::getThetaC(x_dash_, y_dash_, steering_dir_);

            /*if(std::fabs(theta_c_) < 0.001) 
            {

                ROS_ERROR("theta_c ===> 0.0001");
                found_ = true;
                break;

            }*/

            if(yaw_a_ == yaw_b_)
            {
                
                ROS_WARN("===========================================");
                ROS_ERROR("yaw_a_: %f yaw_b_: %f", yaw_a_ * 180 / M_PI, yaw_b_ * 180 / M_PI);
                ROS_ERROR("x_dash_: %f y_dash_: %f", x_dash_, y_dash_);
                ROS_ERROR("theta_dash_: %f", theta_dash_ * 180 / M_PI); 
                ROS_ERROR("theta_c_: %f", theta_c_ * 180 / M_PI);
                ROS_INFO("===========================================");

            }

            if(std::fabs(theta_dash_ - theta_c_) < Constants::Planner::theta_tol_)
            {   
                ROS_WARN(" ======== cnt_ ==> %d ================", cnt_);
                //ROS_INFO("(x_dash_, y_dash_): (%f,%f)", x_dash_, y_dash_);
                //ROS_INFO("theta_dash_ => %f theta_c => %f", theta_dash_ * 180.f / M_PI, theta_c_ * 180.f / M_PI);   
                
                geometry_msgs::Pose or_;
                or_.position.x = xa_; 
                or_.position.y = ya_; 
                or_.orientation = Utils::getQuatFromYaw(yaw_a_);

                geometry_msgs::Pose oc_; 
                oc_.position.x = xb_; 
                oc_.position.y = yb_; 
                oc_.orientation = Utils::getQuatFromYaw(yaw_b_);

                cnt_++;
                
                //ROS_INFO("r_: %f", r_);
                // /ROS_INFO("yaw_a_: %f === yaw_b_: %f", yaw_a_ * 180.f / M_PI, yaw_b_ * 180.f / M_PI);
                
                //found_ = true;

                //visualize_.drawPoint(or_, "point_a");
                //visualize_.drawPoint(oc_, "point_b");
                //ROS_WARN("or_: (%f,%f) oc_: (%f,%f)", or_.position.x, or_.position.y, oc_.position.x, oc_.position.y);
                connectConfigurationToRobot(or_, oc_ , "or_" + std::to_string(cnt_), "oc_" + std::to_string(cnt_), "sc_" + std::to_string(cnt_));
                //break;  
            }

            else 
            {   
                //ROS_INFO("theta_dash_: %f theta_c_: %f" , theta_dash_ * 180.f / M_PI, theta_c_ * 180.f / M_PI);
                //const float del_theta_ = std::fabs(theta_dash_ - theta_c_) * 180.f / M_PI;            
                //ROS_DEBUG("yaw_a_: %f === yaw_b_: %f del_theta_: %f", yaw_a_ * 180.f / M_PI, yaw_b_ * 180.f / M_PI, del_theta_);

            }

        }

        //break;
    }

    //ROS_WARN("cnt_: %d", cnt_);

    return true; 

}

long long int PRM::SimplePRM::set_N()
{

    float w_ = (Constants::MapMetaData::width_/Constants::MapMetaData::res_);
    float h_ = (Constants::MapMetaData::height_/Constants::MapMetaData::res_);

    ll n_ = 1ll * w_ * h_;

    ROS_INFO("n_: %ld", n_);

    return n_;

}

bool PRM::SimplePRM::generateRoadMap()
{
   // ROS_INFO("Inside PRM::plan()");

    //setN();
    //setSR();  //set neighbour search radius

    N_ = 30;
    sr_ = Constants::Planner::max_res_;
    

    //generateSamplePoints(); 
    //buildKDtree();
    //geometry_msgs::Pose pose_;
    //Node2d node_a_(0, 0), node_b_(0, 0);

    Node3d a_(0.f,0.f,3), b_(0.f,0.f,29);

    ROS_INFO("a_ => (%f,%f,%d,%f)", a_.x_, a_.y_, a_.theta_idx_, a_.theta_);
    ROS_INFO("b => (%f,%f,%d,%f)", b_.x_, b_.y_, b_.theta_idx_, b_.theta_);
    


    //generateEdges(node_a_, node_b_);
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

    if(Constants::MapMetaData::res_ > Constants::Planner::max_res_) {

        ROS_ERROR("map_res_ >  max_res! ==> Need to increase map resolution!");

    }
    
    grid_ = map_;

    ROS_DEBUG("MAP PARAMS ===>");
    ROS_DEBUG("dimensions: (%d, %d)", map_->info.height, map_->info.width);
    ROS_DEBUG("resolution: %f", map_->info.resolution);
    ROS_DEBUG("origin: (%f, %f)", map_->info.origin.position.x, map_->info.origin.position.y);

    ROS_WARN("r_min_: %f", Constants::Vehicle::R_MIN_);

    map_set_ = true; 
    
    
}


bool PRM::SimplePRM::connectConfigurationToRobot(   geometry_msgs::Pose or_ , geometry_msgs::Pose oc_, \
                                                    const std::string or_topic_, const std::string oc_topic_, \
                                                    const std::string sc_topic_) 
{

    ROS_INFO("connectConfigToRobot called ==> or_topic_: %s oc_topic: %s sc_topic: %s" , or_topic_.c_str(), oc_topic_.c_str(), sc_topic_.c_str());
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
    geometry_msgs::PoseArray sc_poses_ = generateSteeringCurve(or_,  R_);

    visualize_.publishT<geometry_msgs::PoseArray>(sc_topic_, sc_poses_);

    visualize_.drawPoint(or_, or_topic_);
    visualize_.drawPoint(oc_, oc_topic_);
    
    //add yaw
    //visualize_.drawPoint(x_, y_); //
    
    return true; 

}

bool PRM::SimplePRM::generateSamplePoints()
{

    ROS_INFO("Inside simplePRM::samplePoints!");

    const int w_ = Constants::MapMetaData::width_; 
    const int h_ =  Constants::MapMetaData::height_;

    ROS_INFO("map_dimension: (%d,%d)", h_, w_);

    //range of x in real world and NOT GRID
    std::vector<float> rx_ = {Constants::MapMetaData::origin_x_ , Constants::MapMetaData::origin_x_ + (w_ + 0.5f) * Constants::MapMetaData::res_}; 
    std::vector<float> ry_ = {Constants::MapMetaData::origin_y_ , Constants::MapMetaData::origin_y_ + (h_ + 0.5f) * Constants::MapMetaData::res_} ;
    
    
    std::random_device rd;
    std::mt19937 gen(rd());
    
    std::uniform_real_distribution<float> dist_x(rx_[0], rx_[1]);
    std::uniform_real_distribution<float> dist_y(ry_[0], ry_[1]);
    
    //std::vector<std::vector<int> > pose_flag_(2000, std::vector<int>(2000, -1));

    //std::unordered_set<Node2d> sampled_points_;
    
    //TODO ==> initialise on HEAP
    std::unordered_set<Node2d, Node2dHash> sampled_points_;

    
    nodes2d_.clear();
    nodes2d_.reserve(N_);

    
    
    while(sampled_points_.size() < N_ && ros::ok())  
    {
        //ROS_INFO("nodes2d_.size(): %d", nodes2d_.size());
        const float  x_ = dist_x(gen);
        const float  y_ = dist_y(gen);

        ROS_INFO("(x,y) => (%f,%f)", x_, y_);

        const Node2d node_{x_, y_};

        if(sampled_points_.find(node_) == sampled_points_.end())
        {

            sampled_points_.insert(node_);

        }
        
        nodes2d_.push_back(node_);
    }
    

    ROS_DEBUG("nodes2d_.size(): %d", nodes2d_.size());
    
    geometry_msgs::PoseArray pose_array_;
    pose_array_.header.frame_id = "map"; 
    pose_array_.header.stamp = ros::Time::now(); 



    for(const auto &t: nodes2d_) 
    {

        geometry_msgs::Pose pose_; 
        
        pose_.position.x = t.x_;
        pose_.position.y = t.y_; 
        pose_.orientation = Utils::getQuatFromYaw(0.f);

        pose_array_.poses.push_back(pose_);

    }   

   //visualize_.visualizeSampledPoints(pose_array_);

    visualize_.publishT<geometry_msgs::PoseArray>("sampled_points", pose_array_);

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

        //generateSteeringCurve(rp_, del_);

        r_.sleep(); 
    }



    geometry_msgs::PoseArray pose_array_ob_; 
    pose_array_ob_.header.frame_id = "map"; 
    pose_array_ob_.header.stamp = ros::Time::now();
    pose_array_ob_.poses = std::move(steering_curve_family_poses_);

    //visualize_.visualizeSteeringCurve(pose_array_ob_);
    visualize_.publishT<geometry_msgs::PoseArray>("steering_curve_family", pose_array_ob_);

// /    return true;


}

//TODO ==> fix bug
//generate steering curve points for a particular delta

//generate steering curve points for a particular delta with a config pose
geometry_msgs::PoseArray PRM::SimplePRM::generateSteeringCurve(geometry_msgs::Pose rp_, const float R_)
{
    
    ROS_ERROR("generateSteeringCurve called with R_: %f", R_);
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
    
    //assuming δ > 0
    
  //  ROS_WARN("Case 1 ===>");
    // /ROS_WARN("R_: %f" , R_);
    
    if(R_ > 0.f)
    {

        for (float x_ = 0.f ;  ; x_ += 0.1f)
        {
        
            float y_ ; 
            //if(delta_ > 0.f){


            y_ = -sqrt(pow(R_, 2) - pow(x_ + Constants::Vehicle::a2_,2)) + sqrt(pow(R_,2 ) - pow(Constants::Vehicle::a2_, 2));
            
            if(std::isnan(y_)) {
            
                ROS_WARN("Last x_ value: %f", x_);
                break;
            }
        
            V_ab_pos_.emplace_back(x_,y_);
            //V_ab_zero_.emplace_back(x_, 0);

            //ROS_INFO("(x,y) ==> (%f,%f)", x_, y_);
        
            y_ = sqrt(pow(R_, 2) - pow(x_ + Constants::Vehicle::a2_,2)) - sqrt(pow(R_,2 ) - pow(Constants::Vehicle::a2_, 2));
            
            if(std::isnan(y_)) {
            
               // ROS_WARN("Last x_ value: S%f", x_);
                break;
            }
        
            V_ab_neg_.emplace_back(x_,y_);
            //V_ab_zero_.emplace_back(x_, 0);
        }

       // ROS_WARN("Case 2 ===>");

        for (float x_ = 0.f ;  ; x_ -= 0.1f)
        {
            
            float y_ ; 
            //if(delta_ > 0.f){


            y_ = -sqrt(pow(R_, 2) - pow(x_ + Constants::Vehicle::a2_,2)) + sqrt(pow(R_,2 ) - pow(Constants::Vehicle::a2_, 2));
            
            if(std::isnan(y_)) {
            V_ab_pos_.reserve(1000);
        
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

        for(float x_ =0 ; x_ < Constants::Planner::max_res_; x_+= 0.1f)
        {

            V_ab_zero_.emplace_back(x_, 0);
        }

        for(float x_ =0 ; x_ > -Constants::Planner::max_res_; x_-= 0.1f)
        {

            V_ab_zero_.emplace_back(x_, 0);
        }

    }
   


    //for(auto t: V_ab_) std::cout << t(0) << " " << t(1) << std::endl;

    ///return;

    const int sz_ = ((int)V_ab_pos_.size() +  (int)V_ab_neg_.size() + (int)V_ab_zero_.size());

    //std::vector<Eigen::Matrix3f> V_ob_;

    std::vector<geometry_msgs::Pose> poses_ob_;

    poses_ob_.reserve(sz_ + 100);

    for(const auto &t: V_ab_pos_){
        
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

    for(const auto &t: V_ab_neg_){
        
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

    for(const auto &t: V_ab_zero_){
        
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


    geometry_msgs::PoseArray pose_array_ob_; 
    pose_array_ob_.header.frame_id = "map"; 
    pose_array_ob_.header.stamp = ros::Time::now();
    pose_array_ob_.poses = std::move(poses_ob_);

    

    //visualize_.publishT<geometry_msgs::PoseArray>("steering_curve", pose_array_ob_);
    
    steering_curve_family_poses_.insert(steering_curve_family_poses_.end(), \
                                        std::make_move_iterator(pose_array_ob_.poses.begin()),   \
                                        std::make_move_iterator(pose_array_ob_.poses.end()));
    

    return pose_array_ob_;

    //visualize_.publishT<geometry_msgs::PoseStamped>("point_pose", cp_stamped_);
    //visualize_.visualizePointPose(cp_stamped_);
    //return true;




}



