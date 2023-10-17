#include <non-holonomic-prm-planner/rrt.h>
#include <non-holonomic-prm-planner/path_generator.h>

#include <random>
#include <cmath>

#include <geometry_msgs/PoseArray.h>

extern std::shared_ptr<PRM::RobotModel> robot_;

PRM::rrt::rrt()
{
    //polygon_ = Polygon();
    //polygon_.outer().push_back(point_t(0, 0));
    //polygon_.outer().push_back(point_t(0, 1));
    //polygon_.outer().push_back(point_t(1, 1));
    //polygon_.outer().push_back(point_t(1, 0)

    reset();

    start_pose_sub_ = nh_.subscribe("/initialpose", 1, &rrt::initialPoseCb, this);
    goal_pose_sub_ = nh_.subscribe("/goal", 1, &rrt::goalPoseCb, this);
    rrt_polygon_sub_ = nh_.subscribe("/rviz_polygon", 1, &rrt::polygonCb, this);
    rrt_tree_pub_ = nh_.advertise<geometry_msgs::PoseArray>("rrt_tstart_rtree_ree", 1, true);
    start_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("test_start_pose", 1, true);
    goal_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("test_goal_pose", 1, true);
    circle_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("circle_pose", 1, true);
    arc_end_points_pub_ = nh_.advertise<geometry_msgs::PoseArray>("arc_end_points", 1, true);
    circle_centers_pub_ = nh_.advertise<geometry_msgs::PoseArray>("circle_centers", 1, true);
}

void PRM::rrt::initialPoseCb(geometry_msgs::PoseWithCovarianceStampedConstPtr pose_)
{
    ROS_WARN("========== START POSE RECEIVED ==============");     
    //setting start index in collision detection polygon for testing
    CollisionDetectionPolygon &p = robot_->getCollisionPolyRef();
    bool flag = p.selectCurrentIndex(Point_t{test_start_pose_.pose.position.x, test_start_pose_.pose.position.y}, \
                         Point_t{test_goal_pose_.pose.position.x, test_goal_pose_.pose.position.y});
    if(!flag)
    {
        ROS_ERROR("unable to select current index!");
        return;
    }
    test_start_pose_.header.frame_id= "map" ; 
    test_start_pose_.header.stamp = ros::Time::now(); 
    test_start_pose_.pose.position.x = pose_->pose.pose.position.x; 
    test_start_pose_.pose.position.y = pose_->pose.pose.position.y;
    test_start_pose_.pose.orientation = pose_->pose.pose.orientation;   
    start_pose_set_ = true; 
    start_pose_pub_.publish(test_start_pose_);

    ROS_INFO("max_res: %f", Constants::Planner::max_res_);
    
    rrt_nodePtr root_ = std::make_shared<rrt_node>();  //first node of start rrt 
    root_->pose_ = Pose_{test_start_pose_};
    //extendNode(root_, Pose_{test_goal_pose_}, Constants::Planner::max_res_);
    getNodeExtensions(root_, Constants::Planner::max_res_);
    //return; 

    //point_t center = getCircleCenter(Pose_{test_start_pose_}, 3.f, true);

    /*geometry_msgs::PoseStamped circle_pose_;
    circle_pose_.header.frame_id = "map";
    circle_pose_.header.stamp = ros::Time::now();
    circle_pose_.pose.position.x = bg::get<0>(center);
    circle_pose_.pose.position.y = bg::get<1>(center);
    circle_pose_.pose.orientation = test_start_pose_.pose.orientation;
    circle_pose_pub_.publish(circle_pose_);
    */
}

void PRM::rrt::goalPoseCb(geometry_msgs::PoseStampedConstPtr pose_)
{
    ROS_WARN("========== GOAL POSE RECEIVED =============="); 
    test_goal_pose_ = *pose_;
    goal_pose_set_ = true; 
    goal_pose_pub_.publish(test_goal_pose_);    
    if(!start_pose_set_)
    {
        ROS_ERROR("start_pose is not set!");
        return;
    }
    //bool flag_ = plan(test_start_pose_, test_goal_pose_);
    //if(flag_) { ROS_WARN("======= PLAN WAS FOUND ======"); }        
    //return;
    rrt_nodePtr root_ = std::make_shared<rrt_node>();  //first node of start rrt 
    root_->pose_ = Pose_{test_start_pose_};
    //extendNode(root_, Pose_{test_goal_pose_}, Constants::Planner::max_res_);
    //arc_end_points_pub_.publish(arc_end_points_);
}   

Polygon PRM::rrt::getPolygonFromPolygonMsg(const geometry_msgs::PolygonStamped &msg)
{
    Polygon polygon_; 
    for(int i = 0; i < msg.polygon.points.size(); i++)
    {       
        float x = msg.polygon.points[i].x;
        float y = msg.polygon.points[i].y;
        bg::append(polygon_, point_t{x, y});
    }
    bg::correct(polygon_);
    if(bg::is_valid(polygon_)) {
        
        ROS_INFO("polygon is valid!");
        return polygon_; 
    }
    else
    {
        ROS_ERROR("polygon is not valid!");
        return Polygon();
    }
}

void PRM::rrt::polygonCb(geometry_msgs::PolygonStampedConstPtr msg)
{
    ROS_WARN("========== POLYGON RECEIVED ==============");  
    Polygon polygon_ = getPolygonFromPolygonMsg(*msg);
    if(polygon_.outer().size() > 0)
    {
        rrt_polygon_ = polygon_;
        polygon_set_ = true; 
    }
    else
    {
        ROS_ERROR("[polygonCb] => polygon is not valid!");
        return;
    }
}

void PRM::rrt::reset()
{
    goal_pose_set_ = false; 
    start_pose_set_ = false;
    polygon_set_ = false;

    start_rrt_.clear(); 
    goal_rrt_.clear(); 

    start_rtree_.clear(); 
    goal_rtree_.clear(); 

    //start_rrt_map_.clear(); 
    //goal_rrt_map_.clear();
}

bool PRM::rrt::sampleRandomPoint(const Polygon &polygon, PRM::Pose_ &pose)
{   
    bool is_valid_ = bg::is_valid(polygon);
    if(!is_valid_){ 
        ROS_ERROR("[sampleRandomPoint] => polygon is not valid!");
        return false;
    }

    Box envelope;
    bg::envelope(polygon, envelope);

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> distX(bg::get<bg::min_corner, 0>(envelope), bg::get<bg::max_corner, 0>(envelope));
    std::uniform_real_distribution<double> distY(bg::get<bg::min_corner, 1>(envelope), bg::get<bg::max_corner, 1>(envelope));
    std::uniform_real_distribution<double> disTheta(0, 2 * M_PI);

    int iter_limit_ = 100 * 100 * 100; 
    int num_iters_  = 0 ;

    while (ros::ok() && num_iters_++ < iter_limit_) 
    {
        point_t randomPoint(distX(gen), distY(gen));
        if (bg::within(randomPoint, polygon)) {
            float x = bg::get<0>(randomPoint);
            float y = bg::get<1>(randomPoint);
            float theta = disTheta(gen);

            auto obb_ = robot_->getOBB({x,y}, theta); 
            if(robot_->isConfigurationFree(obb_))
            {
                pose = Pose_{x,y,theta};
                return true; 
            }
        }
    }
    return false; 
}

void PRM::rrt::publishTree(const std::vector<PRM::rrt_nodePtr> &tree_)
{
    geometry_msgs::PoseArray tree_msg_;
    tree_msg_.header.frame_id = "map";
    tree_msg_.header.stamp = ros::Time::now();
    for(auto t: tree_)
    {
        geometry_msgs::Pose pose_;
        pose_.position.x = t->pose_.x;
        pose_.position.y = t->pose_.y;
        pose_.orientation = tf::createQuaternionMsgFromYaw(t->pose_.theta);
        tree_msg_.poses.push_back(pose_);   
    }
    rrt_tree_pub_.publish(tree_msg_);
}


void PRM::rrt::printNode(const rrt_nodePtr &node)
{
    ROS_INFO("============================================");
    ROS_INFO("pose: (%f,%f,%f)", node->pose_.x, node->pose_.y, node->pose_.theta);
    ROS_INFO("cost: %f", node->cost_);
    ROS_INFO("parent: (%f,%f,%f)", node->parent_->pose_.x, node->parent_->pose_.y, node->parent_->pose_.theta);
    ROS_INFO("============================================");
}

// returns the closest node in the tree to the given pose
bool PRM::rrt::getClosestNode(  const RTree &rtree, \
                                const PoseToNodeMap &pose_to_node_map, \
                                const Pose_ &pose, rrt_nodePtr &closest_node)
{
    std::vector<point_t> closest_points_;
    rtree.query(bgi::nearest(point_t{pose.x, pose.y}, 1), std::back_inserter(closest_points_));
    if(closest_points_.size() == 0) { 
        
        ROS_ERROR("No closest point found in rtree!");
        return false; 
    }
    point_t closest_point_ = closest_points_[0];
    auto closest_node_ = pose_to_node_map.find(closest_point_);
    if(closest_node_ == pose_to_node_map.end()) { 
        
        ROS_ERROR("No closest node found in pose_to_node_map! ==> Something is wrong!");
        return false; 
    }
    closest_node = closest_node_->second;
    return true; 
}

geometry_msgs::Pose PRM::rrt::poseFromPose_(const Pose_ pose)
{
    geometry_msgs::Pose pose_;
    pose_.position.x = pose.x;
    pose_.position.y = pose.y;
    pose_.orientation = tf::createQuaternionMsgFromYaw(pose.theta);
    return pose_;
}

Eigen::Matrix3f PRM::rrt::getHomogeneousMatrixFromPose(const Pose_ &pose)
{
    const float x = pose.x;
    const float y = pose.y;
    const float theta = pose.theta;
    const Eigen::Matrix3f &transformation = (Eigen::Matrix3f(3,3) <<    std::cos(theta),-std::sin(theta), x, \
                                                                        std::sin(theta), std::cos(theta), y,\
                                                                        0, 0, 1).finished();
    return transformation;
}

//returns the end points for the nearest node after extendinf it by arc_len 
std::vector<PRM::Pose_> PRM::rrt::getNodeExtensions(const rrt_nodePtr &nearest_node, const float arc_len)
{   
    // ============== TESTING ====================================
    geometry_msgs::PoseArray arc_end_points; 
    arc_end_points.header.frame_id = "map";
    arc_end_points.header.stamp = ros::Time::now();

    geometry_msgs::PoseArray circle_centers;
    circle_centers.header.frame_id = "map";
    circle_centers.header.stamp = ros::Time::now();
    // ============================================================
    
    const float dx = 0.15;
    std::vector<Pose_> node_extensions_; //contains end points for the node extensions
    float ddelta = 5 * M_PI / 180.0;  //step size for delta
    float a2 = Constants::Vehicle::a2_;
    
    for(float de = -Constants::Vehicle::delta_max_; de < Constants::Vehicle::delta_max_; de += ddelta) //simulating delta 
    {   
        if(std::fabs(de) > 0.01)
        {   
            float xr, yr, thetar; // pose in robot frame
            int num_iter =0 ; 
            for(float xr = 0; ;  xr += dx) //simulating x in robot frame
            {   
                float r = Utils::getR(de);
                float yr = -sqrt(pow(r, 2) - pow(xr + a2,2)) + sqrt(pow(r,2 ) - pow(a2, 2));
                if(std::isnan(yr)) {break;}
                if(de <0) {yr *= -1.f ;}
                float thetar = Utils::getThetaC(xr, yr, yr);
                //robot pose in world frame
                const Eigen::Matrix3f  &M_wr = Utils::getHomogeneousTransformationMatrix(Eigen::Vector2f( nearest_node->pose_.x , nearest_node->pose_.y), \
                                                                                        nearest_node->pose_.theta);
                //pose in robot frame
                const Eigen::Matrix3f &M_rp = Utils::getHomogeneousTransformationMatrix(Eigen::Vector2f(xr, yr), thetar);
                //pose in world frame
                const Eigen::Matrix3f &M_wp = M_wr * M_rp;
                
                const Pose_ &node_extension{M_wp(0,2), M_wp(1,2), std::atan2(M_wp(1,0), M_wp(0,0))}; //pose of the extended node                
                
                //bounding box for the extended node + collision check
                const auto &obb = robot_->getOBB({node_extension.x, node_extension.y}, node_extension.theta);
                if(!robot_->isConfigurationFree(obb)) {break; }
                
                node_extensions_.push_back(node_extension);
                arc_end_points.poses.push_back(poseFromPose_(node_extension));
                
                if(norm(xr,yr) > Constants::Planner::max_res_) {break;}
                if(!ros::ok()) {break;}
            }
        }
        else
        {
            //delta is almost zero 
        }
    }
    arc_end_points_pub_.publish(arc_end_points);
    return node_extensions_; 
}



bool PRM::rrt::plan(const geometry_msgs::PoseStamped &start_pose_, const geometry_msgs::PoseStamped &goal_pose_)
{   
    if(!goal_pose_set_ || !start_pose_set_ || !polygon_set_)
    {
        ROS_ERROR("start or goal pose or rrt_polygon is not set!");
        return false;
    }

    Polygon polygon_ = rrt_polygon_;
    CollisionDetectionPolygon &p = robot_->getCollisionPolyRef(); 
    bool index_found = p.selectCurrentIndex(Point_t(start_pose_.pose.position.x, start_pose_.pose.position.y), \
                                            Point_t(goal_pose_.pose.position.x, goal_pose_.pose.position.y));

    if(!index_found) 
    {
        ROS_ERROR("unable to select current index!");
        return false;
    }

    Pose_ goal_{goal_pose_};
    Pose_ start_{start_pose_};

    point_t start_pt{start_.x, start_.y};
    point_t goal_pt{goal_.x, goal_.y};

    // =======================================================================================
    // ============================ RRT FROM START POSE ======================================
    // =======================================================================================
    rrt_nodePtr start_root_= std::make_shared<rrt_node>();  //first node of start rrt
    start_root_->parent_ = nullptr; 
    start_root_->pose_ = Pose_{start_pose_};
    start_root_->cost_ = 0 ;    
    start_rrt_.push_back(start_root_);  //updating start rrt
    start_rtree_.insert(point_t{start_.x, start_.y}); //updating start rtree

    auto start_time = std::chrono::high_resolution_clock::now();

    while(ros::ok())
    {
        Pose_ nxt_pose; 
        bool found; 
        found = sampleRandomPoint(polygon_, nxt_pose);   
        if(!found)
        {
            ROS_ERROR(" === Could not sample a random point ==> Trying again === "); 
            return false;
        }
        
        rrt_nodePtr closest_node_; 
        found = getClosestNode(start_rtree_, start_rrt_map_, nxt_pose, closest_node_);
        if(!found) {
            ROS_ERROR("No closest node found! ==> Breaking while loop!"); 
            break;
        }
        
        /*found = extendNode(closest_node_, nxt_pose);
        if(!found)
        {
            ROS_ERROR("Can't extend the closest node ==> needs to be deleted");
            //deleteNode(closest_node_);
        }*/
    }

    auto end_time = std::chrono::system_clock::now();
    auto elapsed  = std::chrono::duration_cast<std::chrono::seconds>(end_time - start_time);
    
    ROS_WARN("================================================================================") ;
    ROS_WARN("RRT converged in %d seconds", elapsed.count());
    ROS_WARN("================================================================================") ;
    
    return true; 
}




