#include <non-holonomic-prm-planner/roadmap.h>
#include <non-holonomic-prm-planner/helpers.h>
#include <non-holonomic-prm-planner/path_generator.h>



#include <random>
#include <Eigen/Dense>
#include <eigen3/Eigen/Core>


#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>

#include <cmath>
#include <typeinfo>
#include <chrono>
#include <queue>





//forward declaration
float PRM::Constants::MapMetaData::cell_size_; 
float PRM::Constants::MapMetaData::res_; 
float PRM::Constants::MapMetaData::origin_x_; 
float PRM::Constants::MapMetaData::origin_y_;  
int PRM::Constants::MapMetaData::height_; 
int PRM::Constants::MapMetaData::width_; 


int PRM::Roadmap::edge_cnt_ = 0 ;


PRM::Roadmap::Roadmap()
{   

    ROS_WARN("SimplePRM constructor called");
    //cdp_.initialize();

    //visualize_ = std::make_shared<Visualize>();
}


void PRM::Roadmap::clickedPointCb(geometry_msgs::PointStampedConstPtr pose_)
{

    ROS_ERROR("========= CLICKED PT CB ===============  ");

    /*for(auto t: G_)
    {
Inside 
        const NodePtr_ node_ = t.second; 

        ROS_DEBUG("SOURCE NODE ==>");
        //node_->print();

        ROS_DEBUG("DESTINATION NODE ==>");
        for(auto t : *node_->edges_)
        {
            
            const NodePtr_ dst_ = t.node_;
            //dst_->print();
        }

    }*/

    ROS_INFO("[%f,%f]", pose_->point.x, pose_->point.y);


}


void PRM::Roadmap::initialPoseCb(geometry_msgs::PoseWithCovarianceStampedConstPtr pose_)
{

    ROS_WARN("========== START POSE RECEIVED =============="); 

    int cnt_ =0 ; 
    
   ROS_INFO("start_pose.frame: %s", pose_->header.frame_id.c_str());

    geometry_msgs::PoseStamped p_;
    p_.pose = pose_->pose.pose;
    p_.header.frame_id = "map"; 
    p_.header.stamp = ros::Time::now();
    
    //ROS_INFO("start_yaw_: %f", tf::getYaw(pose_->pose.pose.orientation));

    float yaw_ = tf::getYaw(pose_->pose.pose.orientation); 

    if(yaw_ < 0)
    {
        yaw_  += 2 * M_PI;
    }
    
    sp_ = {p_.pose.position.x, p_.pose.position.y, yaw_ / Constants::Planner::theta_sep_};
   //sp_ = {-827.245544,-140.275894,17};
   
   // sp_ = {-838.735840,-141.841537,55};
    
    /*geometry_msgs::PoseStamped p_;
    p_.pose.position.x = sp_.x_; 
    p_.pose.position.y = sp_.y_;
    p_.pose.orientation = Utils::getQuatFromYaw(sp_.theta_);
    p_.header.frame_id = "map"; 
    p_.header.stamp = ros::Time::now();
    */

    ROS_DEBUG("====== INITIAL POSE =================="); 
    //sp_.print();
    
    visualize_->publishT<geometry_msgs::PoseStamped>("start_pose", p_);

    SteeringCurve::generateSteeringCurveFamily(sp_, "initial_pose_family");

    //ROS_INFO("Steering Curve family generated!");

    //sp_.cost_ = 0; 
    //sp_.parent_ = nullptr;

    //ROS_WARN("Printing start pose ==> "); 
    //sp_.print();
    
    const Vec3f &key_ = Utils::getNode3dkey(sp_);
    
    NodePtr_ ptr_; 

    if(G_.find(key_) != G_.end())
    {
        ptr_ = G_[key_];
    }
    else
    {
        ptr_ = std::make_shared<Node3d>(sp_);
    }


    bool flag_ = connectStartPoseToRoadmap(ptr_);

    if(!flag_)
    {
        
        ROS_ERROR("ROADMAP CONNECT FLAG ==> FALSE");
    
    }
    else
    {
        ROS_ERROR("ROADMAP CONNECT FLAG ==> TRUE");
        //ROS_INFO("start_edges_cnt_: %d", G_[Utils::getNode3dkey(sp_)]->edges_->size());
        visualize_->drawNodeNeighbours(ptr_, "inital_pose_neighbours_");
        G_[key_] = ptr_;
        //ROS_INFO("start_edges_cnt_: %d", G_[Utils::getNode3dkey(*ptr_)]->edges_->size());
        

        ROS_DEBUG("G_.size(): %d", G_.size());
    
    }

    //djikstra(ptr_);

    return; 
    
}

void PRM::Roadmap::goalPoseCb(geometry_msgs::PoseStampedConstPtr pose_)
{
    ROS_WARN("========== GOAL POSE RECEIVED =============="); 
    
    
    int cnt_ =0 ; 
    
    //ROS_INFO("start_pose.frame: %s", pose_->header.frame_id.c_str());

    geometry_msgs::PoseStamped p_ = *pose_;
    
    //ROS_INFO("start_yaw_: %f", tf::getYaw(pose_->pose.pose.orientation));

    float yaw_ = tf::getYaw(p_.pose.orientation); 

    if(yaw_ < 0)
    {
        yaw_  += 2 * M_PI;
    }

    
    Node3d gp_ = {p_.pose.position.x, p_.pose.position.y, yaw_ / Constants::Planner::theta_sep_};
    
    visualize_->publishT<geometry_msgs::PoseStamped>("goal_pose", p_);

    
    //generateSteeringCurveFamily(gp_);
    SteeringCurve::generateSteeringCurveFamily(p_.pose);


    const Vec3f &key_ = Utils::getNode3dkey(gp_);
    
    NodePtr_ ptr_; 

    if(G_.find(key_) != G_.end())
    {
        ptr_ = G_[key_];
    }
    else
    {
        ptr_ = std::make_shared<Node3d>(gp_);
    }


    bool flag_ = connectGoalPoseToRoadmap(ptr_);

    if(!flag_)
    {
        
        ROS_ERROR("ROADMAP CONNECT FLAG ==> FALSE");
    
    }
    else
    {
        ROS_DEBUG("ROADMAP CONNECT FLAG ==> TRUE");
    
        G_[key_] = ptr_;
        
        ROS_DEBUG("G_.size(): %d", G_.size());

        NodePtr_ start_ptr_ = G_[Utils::getNode3dkey(sp_)];
        NodePtr_ goal_ptr_ = ptr_;

        int cnt_ = 0 ;


        //std::vector<Node3d> path_ = PathGenerator::getShortestPath(G_, vis_,  start_ptr_, goal_ptr_, visualize_);
        
        
        //ROS_INFO("path_.size(): %d", path_.size());
        //generateROSPath(path_);
        
        bool found_ = PathGenerator::getCollisionFreePath(G_, start_ptr_, goal_ptr_);
        //bool found_ = PathGenerator::checkPathForCollisions(G_, path_, robot_, visualize_);
        
        ROS_WARN("============================================="); 
        ROS_WARN("==========FOUND: %d===========================", found_); 
        ROS_WARN("============================================="); 
        

    }

    //djikstra(ptr_);

    //return; 

}   

void PRM::Roadmap::initialize()
{

    ros::Rate r_(10.0);
    map_sub_ = nh_.subscribe(Constants::map_topic, 1, &Roadmap::setMapCb, this);

    
    while(ros::ok() && !map_set_){

        ROS_DEBUG("Waiting for map!!");
        ros::spinOnce(); 
    
        r_.sleep();
    }

    ROS_WARN("map_set_ is true!");

    // ===============================================================================
    // ======================SUBSCRIBER CALLBACKS=======================================
    // ===============================================================================
    

    start_pose_sub_ = nh_.subscribe("/initialpose", 1, &Roadmap::initialPoseCb, this);
    goal_pose_sub_ = nh_.subscribe("/goal", 1, &Roadmap::goalPoseCb, this);
    clicked_pt_sub_ = nh_.subscribe("/clicked_point", 1, &Roadmap::clickedPointCb, this);
    


    sampler_ = std::make_shared<Sampler>();
    //sampledPoints2D_ = sampler_->generate2DSamplePoints();

}

PRM::NodePtr_ PRM::Roadmap::getNodePtr(const Node3d &node_)
{   
    NodePtr_ ptr_; 

    const Vec3f &key_ = Utils::getNode3dkey(node_);

    if(G_.find(key_) == G_.end())
    {
        ptr_ = std::make_shared<Node3d>(node_);
        G_[key_] = ptr_; 

    }

    return G_[key_];

}

bool PRM::Roadmap::connectStartPoseToRoadmap(NodePtr_ &node_)
{   
    
    ROS_INFO("ConnectToRoadmap function!");

    kdTree::point_t pt_{node_->x_, node_->y_};

    kdTree::pointVec pts_ = kdTree_->neighborhood_points(pt_, Constants::Planner::max_res_);

    ROS_INFO("pts_.size(): %d", pts_.size());

    if((int)pts_.size() == 0)
    {
        ROS_ERROR("Unable to connect start to the roadmap!");
        return false; 
    }

    //const Node3d a_{start_.position.x , start_.position.y, tf::getYaw(start_.orientation) / M_PI};

    bool flag_ = false;
    
    int pnt_cnt_ =0 ; 

    int conn_cnt_ =0 ; 
    
    for(const auto pt_: pts_)
    {
        //ROS_INFO("pnt_cnt_: %d" , pnt_cnt_);
        
        for(int i_ = 0 ; i_ * Constants::Planner::theta_sep_ < 2 * M_PI; i_++)
        {

            const Vec3f &key_{pt_[0], pt_[1], i_};

            // checking if node exists
            if(G_.count(key_) > 0)
            {

                NodePtr_ b_ptr_ = G_[key_];
                if(connectNodes(node_, b_ptr_))
                {   
                    //generateSteeringCurveFamily(*b_ptr_, "family_goal" + std::to_string(i_));
                    flag_ = true; 
                    conn_cnt_++;
                }
            }
            
        }

    }

    ROS_INFO("flag_: %d", flag_);
    ROS_INFO("conn_cnt_: %d", conn_cnt_);
    return flag_; 
}

bool PRM::Roadmap::connectGoalPoseToRoadmap(NodePtr_ &node_)
{   
    
    ROS_INFO("ConnectToRoadmap function!");

    kdTree::point_t pt_{node_->x_, node_->y_};

    kdTree::pointVec pts_ = kdTree_->neighborhood_points(pt_, Constants::Planner::max_res_);

    ROS_INFO("pts_.size(): %d", pts_.size());

    if((int)pts_.size() == 0)
    {
        ROS_ERROR("Unable to connect start to the roadmap!");
        return false; 
    }

    //const Node3d a_{start_.position.x , start_.position.y, tf::getYaw(start_.orientation) / M_PI};

    bool flag_ = false;
    
    int pnt_cnt_ =0 ; 

    int conn_cnt_ =0 ; 
    
    for(const auto pt_: pts_)
    {
        //ROS_INFO("pnt_cnt_: %d" , pnt_cnt_);
        
        for(int i_ = 0 ; i_ * Constants::Planner::theta_sep_ < 2 * M_PI; i_++)
        {
            const Vec3f &key_{pt_[0], pt_[1], i_};
            
            // checking if node exists
            if(G_.count(key_) > 0)
            {

                NodePtr_ b_ptr_ = G_[key_];
                if(connectNodes(b_ptr_, node_))
                {   
                    SteeringCurve::generateSteeringCurveFamily(*b_ptr_, "family_goal" + std::to_string(i_));
                    flag_ = true; 
                    conn_cnt_++;
                }
            }
            
        }

    }

    ROS_INFO("flag_: %d", flag_);
    ROS_INFO("conn_cnt_: %d", conn_cnt_);
    return flag_; 
}


nav_msgs::Path PRM::Roadmap::generateROSPath(const std::vector<Node3d>&path_)
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

    visualize_->publishT<nav_msgs::Path>("path", ros_path_);
    visualize_->publishT<geometry_msgs::PoseArray>("final_path", final_path_);


    //ROS_INFO("Visualiztion finished!");
    return ros_path_;
}

bool PRM::Roadmap::buildKDtree()
{

    //kdTree_ = std::make_shared<kdTree::KDTree>(kd_pts_);

    //kdTree::KDTree tree_(kd_pts_);

    ROS_INFO("Inside buildKDTree!");

    /*if((int)nodes2d_.size() == 0) {

        ROS_ERROR("nodes2d_ is empty!");
        return false;

    }*/

    kdPoints kd_points_;

    //ROS_DEBUG("nodes2d_.size(): %d", nodes2d_.size());

    int cnt_ =0 ; 
    for(const auto &node_: sampledPoints2D_)
    {
        cnt_++; 
        
        const float x_ = node_.x_ ; 
        const float y_ = node_.y_; 

        kdPoint kp_ = {x_, y_};

        kd_points_.push_back(kp_);
        //break;
        //ROS_DEBUG("cnt_: %d" , cnt_);
        //ROS_INFO("(mx_, my_): (%d,%d) ==> (wx_, wy_): (%f,%f)", mx_, my_, wx_, wy_);
        //ROS_INFO("(wx_, wy): (%f,%f)");

    }

    //ROS_INFO("EXITED!");

    auto start = std::chrono::high_resolution_clock::now();

    ROS_INFO("Building kd_tree started!");
    
    kdTree_ = std::make_shared<kdTree::KDTree>(kd_points_);

    
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> duration = end - start;
    double seconds = duration.count();

    ROS_INFO("Finished building kd tree in %f seconds!", seconds);


    //ROS_INFO("kdTree initialized!");
    //kdTree::KDTree tree_(kd_points_);

   return true; 
}




bool PRM::Roadmap::canConnect(NodePtr_ &a_ptr_, NodePtr_&b_ptr_) 
{
    

   // ROS_INFO("generateEdges  3d called!");
    //const float ox_ = Constants::MapMetaData::origin_x_; 
    //const float oy_ = Constants::MapMetaData::origin_y_;

    //ROS_INFO("ox_: %f", ox_); 
    //ROS_INFO("oy_: %f", oy_);

    //TODO ==> fix for these values
    //const Node3d a_{ox_  + 30.f, oy_ + 30.f, 52};
    //const Node3d b_{ox_ + 28.f, oy_ + 30.f, 11};

   // const Node3d a_{ox_  + 31.f, oy_ + 31.f, 38};
    //const Node3d b_{ox_ + 30.f, oy_ + 30.f, 48};

    //Utils::printNode(a_, "a_");
    //Utils::printNode(b_, "b_");

    //const Node2d &a_ = Node2d{ox_  + 31.f, oy_ + 31.f};
    //const Node2d &b_ = Node2d{ox_ + 30.f, oy_ + 30.f};

    //a_ = Node2d{ox_  + 30.f, oy_ + 30.f};
    //b_ = Node2d{ox_ + 32.f, oy_ + 32.f};

  // ROS_WARN("Inside canConnect function!");
    const float dis_ = Utils::euclidean(*a_ptr_, *b_ptr_) ;

    if(dis_ > Constants::Planner::max_res_)
    {      
        //ROS_ERROR("dis_: %f Planner::max_res_: %f", dis_, Constants::Planner::max_res_);
       // ROS_ERROR("euclidean distance  > max_res!");
        return false;
    }

    if(dis_ < 0.001f)
    {      
        //ROS_ERROR("dis_: %f Planner::max_res_: %f", dis_, Constants::Planner::max_res_);
        //ROS_ERROR("euclidean distance  < 0.001!");
        return false;
    }


    const float xa_ = a_ptr_->x_, ya_ = a_ptr_->y_;
    const float xb_ = b_ptr_->x_, yb_ = b_ptr_->y_;

    
    const Vec2f V_oa_{xa_, ya_};
    const Vec2f V_ob_{xb_, yb_};
    
    const float yaw_a_ = a_ptr_->theta_idx_ * Constants::Planner::theta_sep_; 
    const float yaw_b_ = b_ptr_->theta_idx_ * Constants::Planner::theta_sep_;

    
    
    const Mat3f &P_oa_ = (Utils::getHomogeneousTransformationMatrix(V_oa_, yaw_a_));
    const Mat3f &P_ob_ = (Utils::getHomogeneousTransformationMatrix(V_ob_, yaw_b_));

    const Mat3f &P_ao_ = P_oa_.inverse();
            
    const Mat3f &P_ab_ = P_ao_ * P_ob_;  //b in the frame of a
    
    const float x_dash_ = P_ab_(0,2);                                       // Δx in the frame of a 
    const float y_dash_ = P_ab_(1,2);                                       // Δy in the frame of a
    

    if(x_dash_ < 0 && !Constants::Planner::can_reverse_)
    {
        return false;
    }

    float theta_dash_  = std::atan2(P_ab_(1,0), P_ab_(0,0));          // Δtheta in the frame of a
    
    if(theta_dash_ < 0)
    {
        theta_dash_ += 2 *  M_PI; 
    }
    
    const float r_ = Utils::getR(x_dash_, y_dash_);

        
    if(r_ > 0.f && r_ < Constants::Vehicle::R_MIN_)
    {   
        
        // ROS_WARN("yaw_a_: %f === yaw_b_: %f r_: %f", yaw_a_ * 180.f / M_PI, yaw_b_ * 180.f / M_PI, r_);
        return false;
    }

    const float steering_dir_ = Utils::signDelta(x_dash_, y_dash_);
    const float theta_c_  = Utils::getThetaC(x_dash_, y_dash_, steering_dir_);

   // ROS_INFO("theta_dash_ => %f theta_c => %f", theta_dash_ * 180.f / M_PI, theta_c_ * 180.f / M_PI);   
    float dis_cost_, ang_cost_; 


    if(r_ > 0.f)
    {   
        if(x_dash_ > 0)
        {
            
            dis_cost_ =  10 * Constants::Planner::w_dis_ * r_ * theta_dash_;
            ang_cost_ =  Constants::Planner::w_ang_ * theta_dash_ ;
        
        }
        else
        {
            dis_cost_ = Constants::Planner::w_dis_ * r_ * theta_dash_ * 1000;;
            ang_cost_ = Constants::Planner::w_ang_ * theta_dash_;
        
        }
        
    }
    else 
    {
        //implies y_dash is 0 
        
        if(x_dash_ >= 0)
        {   

            ang_cost_ = 0 ; 
            dis_cost_ = Constants::Planner::w_dis_ * x_dash_;
        } 
        else
        {
            //implies reverse movement without turning
            ang_cost_ = 0 ; 
            dis_cost_ = Constants::Planner::w_rev_ * std::fabs(x_dash_);
        }
    }

    if(std::fabs(theta_dash_ - theta_c_) < Constants::Planner::theta_tol_)
    {

        std::shared_ptr<Edge> e_ = std::make_shared<Edge>(b_ptr_, dis_cost_, ang_cost_);
        
        //const std::string key_ = std::to_string(e_->node_->x_) } 
        const Vec3f key_ = Utils::getNode3dkey(*e_->node_);
        a_ptr_->addEdge(key_, e_);
        
        return true; 
    
    }

    return false;
    //return (std::fabs(theta_dash_ - theta_c_) < Constants::Planner::theta_tol_);
    
}


bool PRM::Roadmap::connectNodes(NodePtr_ &a_ptr_,  NodePtr_ &b_ptr_)
{   

    //ROS_INFO("Inside connectNodes function!");

    //std::shared_ptr<Edge> e_;
    //NodePtr_ node_ ;
    
    const Vec3f &a_key_ = Utils::getNode3dkey(*a_ptr_);
    const Vec3f &b_key_ = Utils::getNode3dkey(*b_ptr_);

    
    bool can_connect_ = canConnect(a_ptr_, b_ptr_);
    
    if(can_connect_)
    {   
        int cnt_ = Roadmap::edge_cnt_++;
        //connectConfigurationToRobot(*a_ptr_, *b_ptr_, "or_" + std::to_string(cnt_), "oc_" + std::to_string(cnt_), "sc_" + std::to_string(cnt_));
        
        if(G_.find(b_key_) == G_.end())
        {
            G_[b_key_] = b_ptr_;
        }


        return true; 
    }

    return false; 
}

int PRM::Roadmap::generateEdges(const Node2d &a2_, const Node2d &b2_)
{

    int precision = 10;
    
    for(int a_i_ = 0;  a_i_ * Constants::Planner::theta_sep_ < 2 * M_PI; a_i_++)
    {
        
        const Vec3f &a_key_{a2_.x_, a2_.y_, a_i_};

        const std::array<float, 2> translation_{a2_.x_ , a2_.y_};
        const float heading_ =  a_i_ * Constants::Planner::theta_sep_;
        const std::vector<float> obb_ = robot_->getOBB(translation_, heading_);

        bool is_free_ = robot_->isConfigurationFree(obb_);

        if(!is_free_)
        {
            continue;
        }

        NodePtr_ a_ptr_;

        if(G_.find(a_key_) != G_.end())
        {
            a_ptr_ = G_[a_key_];
        }
        else
        {
            a_ptr_ = std::make_shared<Node3d>(a2_.x_ , a2_.y_, a_i_);
        }
        
        for (int b_i_ = 0.0; b_i_  * Constants::Planner::theta_sep_ < 2 * M_PI; b_i_++)
        {   

            
            const Vec3f &b_key_{b2_.x_, b2_.y_, b_i_};

            const std::array<float, 2> translation_{b2_.x_ , b2_.y_};
            const float heading_ =  b_i_ * Constants::Planner::theta_sep_;
            const std::vector<float> obb_ = robot_->getOBB(translation_, heading_);

            bool is_free_ = robot_->isConfigurationFree(obb_);

            if(!is_free_)
            {
                continue;
            }

            NodePtr_ b_ptr_;

            if(G_.find(b_key_) != G_.end())
            {
                b_ptr_ = G_[b_key_];
            }
            else
            {
                b_ptr_ = std::make_shared<Node3d>(b2_.x_ , b2_.y_, b_i_);
            }

            bool flag_ = connectNodes(a_ptr_, b_ptr_);   

            
            if(flag_)
            {
                //ROS_DEBUG("===================");
                //a3_.print();
                //b3_.print();

            }

            //cnt_ = (flag_ ? cnt_ + 1 : cnt_);         
        }

        /*if(G_.count(a_key_) == 0)
        {
            G_.erase(a_key_);
        }*/

        //ROS_INFO("cnt_: %d", cnt_);
    }

    //ROS_INFO("cnt_: %d", cnt_);
    int del_cnt_ = 0 ;
  
    return 1 ;

}


void PRM::Roadmap::buildGraph()
{   
    
    ROS_INFO("Building Roadmap ===> wait for some time!");

    int cnt_ =0 ;
    //std::cout << "sampledPOints2d.size(): " << (int)sampledPoints2D_.size() << std::endl;
    for(const auto &node_ : sampledPoints2D_)
    {   
        //ROS_DEBUG("build_graph_cnt_: %d", build_graph_cnt_); 
        //std::cout << cnt_++ << std::endl;
        if(!ros::ok())
        {
            break;
        }

        const kdTree::point_t o_{node_.x_, node_.y_};
        
        kdTree::pointVec neighbours_ = kdTree_->neighborhood_points(o_, Constants::Planner::sr_);

        for(const kdTree::point_t pt_: neighbours_)
        {   

            if(!ros::ok())
            {
                break;
            }
            
            const Node2d a_(o_[0], o_[1]);
            const Node2d b_{pt_[0], pt_[1]}; 

            if(a_ == b_) 
            {
                continue;
            }
        
            int cnt_ = generateEdges(a_, b_);
            //ROS_DEBUG("%d edges added", cnt_);
        }
    }   

    ROS_INFO("ROADMAP BUILT!");
    ROS_DEBUG("G_.size(): %d", G_.size());

}

bool PRM::Roadmap::generateRoadMap()
{

    sampledPoints2D_= sampler_->generate2DSamplePoints();
    
    buildKDtree();
    buildGraph();

    int cnt_ = 0; 

    return true; 

}

void PRM::Roadmap::setMapCb(nav_msgs::OccupancyGrid::ConstPtr map_)
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

   // ROS_WARN("r_min_: %f", Constants::Vehicle::R_MIN_);

    map_set_ = true; 
    
    
}