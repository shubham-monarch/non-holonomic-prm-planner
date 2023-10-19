#include <non-holonomic-prm-planner/rrt.h>
#include <non-holonomic-prm-planner/path_generator.h>

#include <random>
#include <cmath>

#include <geometry_msgs/PoseArray.h>

#include <boost/geometry/geometries/ring.hpp>
#include <boost/geometry/algorithms/append.hpp>
#include <boost/geometry/algorithms/for_each.hpp>

extern std::shared_ptr<PRM::RobotModel> robot_;

PRM::rrt::rrt():start_pose_set_{false}, goal_pose_set_{false}, polygon_set_{false}
{
    //polygon_ = Polygon();
    //polygon_.outer().push_back(point_t(0, 0));
    //polygon_.outer().push_back(point_t(0, 1));
    //polygon_.outer().push_back(point_t(1, 1));
    //polygon_.outer().push_back(point_t(1, 0)

    //reset();

    start_pose_sub_ = nh_.subscribe("/initialpose", 1, &rrt::initialPoseCb, this);
    goal_pose_sub_ = nh_.subscribe("/goal", 1, &rrt::goalPoseCb, this);
    rrt_polygon_sub_ = nh_.subscribe("/rviz_polygon", 1, &rrt::polygonCb, this);
    rrt_tree_pub_ = nh_.advertise<geometry_msgs::PoseArray>("rrt_tree", 1, true);
    start_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("test_start_pose", 1, true);
    goal_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("test_goal_pose", 1, true);
    circle_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("circle_pose", 1, true);
    arc_end_points_pub_ = nh_.advertise<geometry_msgs::PoseArray>("arc_end_points", 1, true);
    circle_centers_pub_ = nh_.advertise<geometry_msgs::PoseArray>("circle_centers", 1, true);
    rrt_path_pub_ = nh_.advertise<geometry_msgs::PoseArray>("rrt_path", 1, true);
    rrt_service_ = nh_.advertiseService("rrt_service", &rrt::getPathService, this);
    closest_points_pub_ = nh_.advertise<geometry_msgs::PoseArray>("closest_points", 1, true);
}

void PRM::rrt::initialPoseCb(geometry_msgs::PoseWithCovarianceStampedConstPtr pose_)
{
    ROS_WARN("========== START POSE => (%f,%f) RECEIVED ==============", pose_->pose.pose.position.x, pose_->pose.pose.position.y);     
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

    /*//====== TESTING ======
    ROS_INFO("Starting testing!");
    rrt_nodePtr st_node_ = std::make_shared<rrt_node>();
    st_node_->pose_ = test_start_pose_;
    getNodeExtensions(st_node_, Constants::Planner::max_res_, false);
    //=====================
    */
}

void PRM::rrt::goalPoseCb(geometry_msgs::PoseStampedConstPtr pose_)
{
    ROS_WARN("========== GOAL POSE RECEIVED =============="); 
    test_goal_pose_ = *pose_;
    goal_pose_set_ = true; 
    goal_pose_pub_.publish(test_goal_pose_);    
    /*
    if(!start_pose_set_)
    {
        ROS_ERROR("start_pose is not set!");
        return;
    }
    reset();
    bool found = plan(test_start_pose_, test_goal_pose_);    
    ROS_WARN("plan() returned %s", found ? "true" : "false");   
    */
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
    //polygon_set_ = false;

    st_rtree_ = std::make_shared<RTree>();
    go_rtree_ = std::make_shared<RTree>();
    curr_rtree_ = std::make_shared<RTree>();
    
    st_pose_to_node_map_ = std::make_shared<PoseToNodeMap>();
    go_pose_to_node_map_ = std::make_shared<PoseToNodeMap>();
    curr_pose_to_node_map_ = std::make_shared<PoseToNodeMap>();
    
    st_dmap_ = std::make_shared<DMap>();
    go_dmap_ = std::make_shared<DMap>();
    curr_dmap_ = std::make_shared<DMap>();

    rrt_tree_.poses.clear();

    //start_rtree_.clear(); 
    //goal_rtree_.clear(); 
    //start_rrt_map_.clear(); 
    //goal_rrt_map_.clear();
    //srrt_dmap_.clear();
    //rrt_tree_.poses.clear();
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

void PRM::rrt::printNode(const rrt_nodePtr &node)
{
    ROS_INFO("============================================");
    ROS_INFO("pose: (%f,%f,%f)", node->pose_.x, node->pose_.y, node->pose_.theta);
    ROS_INFO("cost: %f", node->cost_);
    ROS_INFO("parent: (%f,%f,%f)", node->parent_->pose_.x, node->parent_->pose_.y, node->parent_->pose_.theta);
    ROS_INFO("============================================");
}

void PRM::rrt::publishRRTPath(const rrt_nodePtr &node)
{
    geometry_msgs::PoseArray rrt_path;
    rrt_path.header.frame_id = "map";
    rrt_path.header.stamp = ros::Time::now();
    rrt_nodePtr curr_node = node;
    while(curr_node != nullptr)
    {
        geometry_msgs::PoseStamped pose_; 
        pose_.pose.position.x = curr_node->pose_.x;
        pose_.pose.position.y = curr_node->pose_.y;
        pose_.pose.orientation = tf::createQuaternionMsgFromYaw(curr_node->pose_.theta);
        rrt_path.poses.push_back(pose_.pose);
        curr_node = curr_node->parent_;
    }
    //ROS_INFO("Publishing RRT Path ==>");
    rrt_path_pub_.publish(rrt_path);
}

// returns the closest node in the rtree to the given pose
bool PRM::rrt::getNearestNodeToSampledPoint(const Pose_ &pose, rrt_nodePtr &closest_node)
{
    std::vector<point_t> closest_points_;
    curr_rtree_->query(bgi::nearest(point_t{pose.x, pose.y}, 1), std::back_inserter(closest_points_));
    if(closest_points_.size() == 0) { 
        ROS_ERROR("No closest point found in rtree!");
        return false; 
    }
    point_t closest_point_ = closest_points_[0];
    auto closest_node_ = curr_pose_to_node_map_->find(closest_point_);
    if(closest_node_ == curr_pose_to_node_map_->end()) { 
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
std::vector<PRM::Pose_> PRM::rrt::getNodeExtensions(const rrt_nodePtr &nearest_node, const float arc_len, const bool fwd)
{   
    ROS_INFO("Generating node extension for (%f,%f)", nearest_node->pose_.x, nearest_node->pose_.y);
    // ============== TESTING ====================================
    geometry_msgs::PoseArray arc_end_points; 
    arc_end_points.header.frame_id = "map";
    arc_end_points.header.stamp = ros::Time::now();

    geometry_msgs::PoseArray circle_centers;
    circle_centers.header.frame_id = "map";
    circle_centers.header.stamp = ros::Time::now();
    // ============================================================
    
    const float dx = (fwd ? 1.f : -1.f) * 0.15 ;
    std::vector<Pose_> node_extensions_;  //contains end points for the node extensions
    node_extensions_.reserve(1000);
    float ddelta = 5 * M_PI / 180.0;  //step size for delta
    float a2 = Constants::Vehicle::a2_;
    int deleted_cnt = 0 ;
    for(float de = -Constants::Vehicle::delta_max_; de < Constants::Vehicle::delta_max_; de += ddelta) //simulating delta 
    {   
        if(std::fabs(de) > 0.01)
        {   
            bool collision = false; //check whether end points is collision free 
            float xr, yr, thetar; // pose in robot frame
            Pose_ node_extension; //pose of the extended node
            int num_iter = 0 ;
            for(float xr = 0; ;  xr += dx, num_iter++) //simulating x in robot frame
            {   
                float r = Utils::getR(de);
                float yr = -sqrt(pow(r, 2) - pow(xr + a2,2)) + sqrt(pow(r,2 ) - pow(a2, 2));
                if(std::isnan(yr)) {break;}
                if(de <0) {yr *= -1.f ;}
                if(norm(xr,yr) > Constants::Planner::max_res_) {break;}
                float thetar = Utils::getThetaC(xr, yr, yr);
                //robot pose in world frame
                const Eigen::Matrix3f  &M_wr = Utils::getHomogeneousTransformationMatrix(Eigen::Vector2f( nearest_node->pose_.x , nearest_node->pose_.y), \
                                                                                        nearest_node->pose_.theta);
                //pose in robot frame
                const Eigen::Matrix3f &M_rp = Utils::getHomogeneousTransformationMatrix(Eigen::Vector2f(xr, yr), thetar);
                //pose in world frame
                const Eigen::Matrix3f &M_wp = M_wr * M_rp;
                node_extension = Pose_{M_wp(0,2), M_wp(1,2), std::atan2(M_wp(1,0), M_wp(0,0))}; //pose of the extended node                
                //bounding box for the extended node + collision check
                const auto &obb = robot_->getOBB({node_extension.x, node_extension.y}, node_extension.theta);
                if(!robot_->isConfigurationFree(obb)) 
                {   
                    collision = true;
                    break;
                }
                if(!ros::ok()) {break;}
            }
            if(collision) { continue; }
            if(curr_dmap_->count({node_extension.x, node_extension.y})) {
                ROS_WARN("Already deleted node was generated again!");
                deleted_cnt++; 
                continue;
            }
            arc_end_points.poses.push_back(poseFromPose_(node_extension));
            node_extensions_.push_back(node_extension);
        }
        else
        {
            //delta is almost zero 
        }
    }
    arc_end_points_pub_.publish(arc_end_points);
    ROS_INFO("Generated %d node extensions", node_extensions_.size());
    return node_extensions_; 
}

PRM::Pose_ PRM::rrt::getClosestNodeExtensionToGoal(const std::vector<Pose_> &poses, const Pose_ &goal_pose)
{
    float mn_dis = std::numeric_limits<float>::max();
    Pose_ closest_pose_;
    for(const auto &pose : poses)
    {
        float dis = euclidDis(pose, goal_pose);
        if(dis < mn_dis)
        {
            mn_dis = dis;
            closest_pose_ = pose;
        }
    }
    return closest_pose_;
}

//TODO => add rtree to function var
bool PRM::rrt::addPoseToTree(const Pose_ &pose, const rrt_nodePtr &parent)
{   
    ROS_INFO("Adding (%f,%f) to the tree", pose.x, pose.y);
    point_t pt{pose.x, pose.y};
    if(curr_pose_to_node_map_->count(pt) > 0) {
        ROS_WARN("Point already present in tree!");
        return false; 
    }
    rrt_nodePtr node = std::make_shared<rrt_node>();
    node->pose_ = pose;
    node->parent_ = parent;
    parent->children_.push_back(node);
    curr_rtree_->insert(pt);
    curr_pose_to_node_map_->insert({point_t{pose.x, pose.y}, node}); //updating map
    
    rrt_tree_.header.frame_id ="map";
    rrt_tree_.header.stamp = ros::Time::now();
    rrt_tree_.poses.push_back(poseFromPose_(pose));    
    rrt_tree_pub_.publish(rrt_tree_);
    return true;
}

bool PRM::rrt::getPathService(prm_planner::PRMService::Request& req, prm_planner::PRMService::Response &res)
{
    Point_t start_t{req.start.pose.pose.position.x, req.start.pose.pose.position.y};
    Point_t goal_t{req.end.pose.position.x, req.end.pose.position.y}; 

    geometry_msgs::PoseStamped start, goal; 
    
    start.header.frame_id = "map";
    start.header.stamp = ros::Time::now(); 
    start.pose.position.x = req.start.pose.pose.position.x; 
    start.pose.position.y = req.start.pose.pose.position.y; 
    start.pose.orientation = req.start.pose.pose.orientation; 

    goal.header.frame_id = "map"; 
    goal.header.stamp = ros::Time::now(); 
    goal.pose.position.x = req.end.pose.position.x;
    goal.pose.position.y = req.end.pose.position.y;
    goal.pose.orientation = req.end.pose.orientation;
    
    start_pose_pub_.publish(start); 
    goal_pose_pub_.publish(goal);
    
    CollisionDetectionPolygon &p = robot_->getCollisionPolyRef();
    if (p.selectCurrentIndex(start_t, goal_t)) //index is cleared in plan()
    {
        if (!req.start_runway.empty()){ p.carveRunway(req.start_runway[0],req.start_runway[1],true); } 
        if (!req.goal_runway.empty()) { p.carveRunway(req.goal_runway[0],req.goal_runway[1],false);  }

        if(!isFree(start)) { ROS_ERROR("start is not free!"); return false; }
        if(!isFree(goal)) { ROS_ERROR("goal is not free!"); return false; } 

        while(ros::ok() && (!polygon_set_ || !start_pose_set_ || !goal_pose_set_)) 
        {   
            
            ROS_ERROR("polygon_set or start_pose_set or goal_pose_set is false!");
            ROS_INFO("polygon_set: %d start_pose_set: %d goal_pose_set: %d", polygon_set_, start_pose_set_, goal_pose_set_);
            ros::Duration(1.0).sleep(); 
            ros::spinOnce();
        }

        bool planned = plan(test_start_pose_, test_goal_pose_);
        
        ROS_DEBUG("======================================================================") ;
        ROS_DEBUG("planned: %d", planned);
        ROS_DEBUG("======================================================================") ;
        
        ros::Duration(10.0).sleep();    
        p.repairPolygons();

        return planned;

        /*auto start_obb_ = robot_->getOBB({start.pose.position.x, start.pose.position.y}, tf::getYaw(start.pose.orientation));
        auto goal_obb_ = robot_->getOBB({goal.pose.position.x, goal.pose.position.y}, tf::getYaw(goal.pose.orientation));
        ROS_INFO("start_collision: %d", robot_->isConfigurationFree(start_obb_));
        ROS_INFO("goal_collision: %d", robot_->isConfigurationFree(goal_obb_));

        while(ros::ok() && !polygon_set_) 
        {
            ROS_ERROR("polygon_set is false!");
            ros::Duration(0.1).sleep(); 
            ros::spinOnce();
        }

        bool planned = plan(start, goal);
        ROS_INFO("planned: %d", planned);
        ros::Duration(10.0).sleep();
        p.repairPolygons();
        return planned;*/
    }
    return false;
}

bool PRM::rrt::isFree(const geometry_msgs::PoseStamped &pose)
{
    auto obb_ = robot_->getOBB({pose.pose.position.x, pose.pose.position.y}, tf::getYaw(pose.pose.orientation));
    return robot_->isConfigurationFree(obb_);
}

//TO-DO => Add collision check
// fwd => direction to connect
//checks whether two poses can be connected by a collision steering curve
bool PRM::rrt::canConnect(const Pose_ &a, const Pose_ &b, const bool fwd)  
{
    const float dis_ = euclidDis(a, b) ;
    if(dis_ > Constants::Planner::max_res_) {return false; }
    //TO-DO => check
    if(dis_ < 0.001f) {return false; }
    
    const float xa_ = a.x, ya_ = a.y, yaw_a_ = a.theta;
    const float xb_ = b.x, yb_ = b.y, yaw_b_ = b.theta;
    
    const Vec2f V_oa_{xa_, ya_};
    const Vec2f V_ob_{xb_, yb_};
    
    const Mat3f &P_oa_ = (Utils::getHomogeneousTransformationMatrix(V_oa_, yaw_a_));
    const Mat3f &P_ob_ = (Utils::getHomogeneousTransformationMatrix(V_ob_, yaw_b_));

    const Mat3f &P_ao_ = P_oa_.inverse();
    const Mat3f &P_ab_ = P_ao_ * P_ob_;  //b in the frame of a
    
    const float x_dash_ = P_ab_(0,2);                                       // Δx in the frame of a 
    const float y_dash_ = P_ab_(1,2);                                       // Δy in the frame of a

    if(x_dash_ < 0 && fwd) {return false; }
    if(x_dash_ > 0 && !fwd) {return false; }

    float theta_dash_  = std::atan2(P_ab_(1,0), P_ab_(0,0));          // Δtheta in the frame of a
    if(theta_dash_ < 0) { theta_dash_ += 2 *  M_PI; }

    const float r_ = Utils::getR(x_dash_, y_dash_);
    if(r_ > 0.f && r_ < Constants::Vehicle::R_MIN_) {return false; }

    const float steering_dir_ = Utils::signDelta(x_dash_, y_dash_);
    const float theta_c_  = Utils::getThetaC(x_dash_, y_dash_, steering_dir_);

    return (std::fabs(theta_dash_ - theta_c_) < Constants::Planner::theta_tol_);
}

bool PRM::rrt::deleteNode(const Pose_ &pose)
{
    bool found;
    found = curr_pose_to_node_map_->count(point_t{pose.x, pose.y}); 
    if(!found)
    {
        ROS_ERROR("Node not found in curr_pose_to_node_map_ ==> Something is wrong!");
        return false;
    }
    
    found = curr_rtree_->count(point_t{pose.x, pose.y});
    if(!found)
    {
        ROS_ERROR("Node not found in curr_rtree_ ==> Something is wrong!");
        return false;
    }

    found = curr_dmap_->count({pose.x, pose.y}); 
    if(found)
    {
        ROS_ERROR("Node found in curr_dmap_ ==> Something is wrong!");
        return false;
    }

    curr_pose_to_node_map_->erase(point_t{pose.x, pose.y});
    curr_rtree_->remove(point_t{pose.x,pose.y});
    curr_dmap_->insert({pose.x, pose.y});
    return true;
}

bool PRM::rrt::plan(const geometry_msgs::PoseStamped &start, const geometry_msgs::PoseStamped &goal)
{   
    /*if(!goal_pose_set_ || !start_pose_set_ || !polygon_set_)
    {
        ROS_ERROR("start or goal pose or rrt_polygon is not set!");
        return false;
    }*/

    ROS_INFO("start: (%f,%f) goal: (%f,%f)", start.pose.position.x, start.pose.position.y, goal.pose.position.x, goal.pose.position.y);
    reset();

    //===================== TESTING ===========
    ROS_WARN("After reset()!"); 
    ROS_WARN("curr_dmap_.size(): %d",curr_dmap_->size());
    ROS_WARN("curr_pose_to_node_map_.size(): %d", curr_pose_to_node_map_->size());
    ROS_WARN("curr_rtree_.size(): %d", curr_rtree_->size());
    //=========================================
    
    Polygon polygon_ = rrt_polygon_;
    CollisionDetectionPolygon &p = robot_->getCollisionPolyRef(); 
    bool index_found = p.selectCurrentIndex(Point_t(start.pose.position.x, start.pose.position.y), \
                                            Point_t(goal.pose.position.x, goal.pose.position.y));

    if(!index_found) 
    {
        ROS_ERROR("unable to select current index!");
        return false;
    }

    Pose_ goal_pose{goal};
    Pose_ start_pose{start};

    point_t start_pt{start_pose.x, start_pose.y};
    point_t goal_pt{goal_pose.x, goal_pose.y};

    // =======================================================================================
    // ============================ RRT FROM START POSE ======================================
    // =======================================================================================
    rrt_nodePtr start_node = std::make_shared<rrt_node>();  //first node of start rrt
    start_node->parent_ = nullptr; 
    start_node->pose_ = start_pose;
    start_node->cost_ = 0 ;    
    st_rtree_->insert(start_pt); //rtree
    st_pose_to_node_map_->insert({start_pt, start_node}); //map

    // =======================================================================================
    // ============================ RRT FROM GOAL POSE =======================================
    // =======================================================================================
    rrt_nodePtr go_node = std::make_shared<rrt_node>();  //first node of start rrt
    go_node->parent_ = nullptr; 
    go_node->pose_ = start_pose;
    go_node->cost_ = 0 ;    
    go_rtree_->insert(start_pt); //rtree
    go_pose_to_node_map_->insert({start_pt, go_node}); //map
    //=========================================================================================
    

    // === setting curr vars ======
    setRtree(go_rtree_);
    setPoseToNodeMap(go_pose_to_node_map_);
    setDMap(go_dmap_);
    // ============================
    auto start_time = std::chrono::high_resolution_clock::now();

    int max_iter = 10000;
    int iter_cnt = 0;

    geometry_msgs::PoseArray closest_points_; 
    closest_points_.header.frame_id = "map";
    closest_points_.header.stamp = ros::Time::now();

    bool goal_reached = false; 

    while(ros::ok() )
    {   
        ROS_WARN("=========================================================================");
        ROS_WARN("iter_cnt: %d", iter_cnt);
        ROS_WARN("curr_pose_to_node_map_.size(): %d curr_rtree_.size(): %d", (int)curr_pose_to_node_map_->size(), (int)curr_rtree_->size());
        ROS_WARN("=========================================================================");

        iter_cnt++; 
        if(iter_cnt >= max_iter) 
        {
            ROS_ERROR("======== REACHED MAX_ITER ========="); 
            break; 
        }

        if((int)curr_pose_to_node_map_->size() != (int)curr_rtree_->size())
        {   
            //ROS_INFO("start_rrt_map_.size(): %d", (int)start_rrt_map_.size());
            //ROS_INFO("start_rtree_.size(): %d", (int)start_rtree_.size());
            //ROS_INFO("iter_cnt: %d", iter_cnt);
            ROS_ERROR("Size mismatch found ==> Something is wrong!");
            return false;
        }

        if(curr_pose_to_node_map_->empty())
        {
            ROS_ERROR("curr_pose_to_node_map is empty==> No node left to exapand!");
            return false;
        }

        Pose_ nxt_pose; 
        bool found; 
        found = sampleRandomPoint(polygon_, nxt_pose);   
        if(!found)
        {
            ROS_ERROR(" === Could not sample a random point ==> Trying again === "); 
            return false;
        }

        rrt_nodePtr closest_node_; 
        found = getNearestNodeToSampledPoint(nxt_pose, closest_node_);
        if(!found) {
            ROS_ERROR("No closest node found! ==> Breaking while loop!"); 
            return false;
        }
        
        closest_points_.poses.push_back(poseFromPose_(closest_node_->pose_));
        closest_points_pub_.publish(closest_points_);
        
        if(euclidDis(closest_node_->pose_, goal_pose) < Constants::Planner::max_res_)
        {
            ROS_WARN("Goal is within max_res_");
            if(canConnect(closest_node_->pose_, goal_pose, false))
            {
                publishRRTPath(closest_node_);
                goal_reached = true;
                break;
                //return true; 
            }
            else
            {
                ROS_ERROR("Unable to connect to the goal pose! ==> deleting closest node");
                deleteNode(closest_node_->pose_);
            }
        }
        
        std::vector<Pose_> node_extensions_ = getNodeExtensions(closest_node_, Constants::Planner::max_res_, false);   
        //TO-DO ==> Add node-deletion logic
        if(node_extensions_.empty()) 
        {   
            //publishRRTPath(closest_node_);
            ROS_ERROR("No node extensions found! ==> CLOSEST NODE IS A DEAD-END ==> deleting!"); 
            deleteNode(closest_node_->pose_);
            continue;
        }
        Pose_ closest_pose_ = getClosestNodeExtensionToGoal(node_extensions_, goal_pose);
        addPoseToTree(closest_pose_, closest_node_);
    }

    auto end_time = std::chrono::system_clock::now();
    auto elapsed  = std::chrono::duration_cast<std::chrono::seconds>(end_time - start_time);
    
    ROS_DEBUG("================================================================================") ;
    ROS_DEBUG("RRT converged in %d seconds", elapsed.count());
    ROS_DEBUG("================================================================================") ;
    
    return goal_reached; 
}




