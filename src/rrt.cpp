#include <non-holonomic-prm-planner/rrt.h>
#include <non-holonomic-prm-planner/path_generator.h>

#include <random>
#include <cmath>

#include <geometry_msgs/PoseArray.h>

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
    rrt_tree_pub_ = nh_.advertise<geometry_msgs::PoseArray>("rrt_tstart_rtree_ree", 1, true);
    start_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("test_start_pose", 1, true);
    goal_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("test_goal_pose", 1, true);
    circle_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("circle_pose", 1, true);
    arc_end_points_pub_ = nh_.advertise<geometry_msgs::PoseArray>("arc_end_points", 1, true);
    circle_centers_pub_ = nh_.advertise<geometry_msgs::PoseArray>("circle_centers", 1, true);
    rrt_path_pub_ = nh_.advertise<geometry_msgs::PoseArray>("rrt_path", 1, true);
    rrt_service_ = nh_.advertiseService("rrt_service", &rrt::getPathService, this);
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
    reset();
    bool found = plan(test_start_pose_, test_goal_pose_);    
    ROS_WARN("plan() returned %s", found ? "true" : "false");   
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
    //goal_pose_set_ = false; 
    //start_pose_set_ = false;
    //polygon_set_ = false;

    //start_rrt_.clear(); 
    //goal_rrt_.clear(); 

    start_rtree_.clear(); 
    goal_rtree_.clear(); 

    start_rrt_map_.clear(); 
    goal_rrt_map_.clear();
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
    std::vector<Pose_> node_extensions_;  //contains end points for the node extensions
    node_extensions_.reserve(1000);
    float ddelta = 5 * M_PI / 180.0;  //step size for delta
    float a2 = Constants::Vehicle::a2_;
    
    for(float de = -Constants::Vehicle::delta_max_; de < Constants::Vehicle::delta_max_; de += ddelta) //simulating delta 
    {   
        if(std::fabs(de) > 0.01)
        {   
            bool collision = false; //check whether end points is collision free 
            float xr, yr, thetar; // pose in robot frame
            Pose_ node_extension; //pose of the extended node
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
                
                node_extension = Pose_{M_wp(0,2), M_wp(1,2), std::atan2(M_wp(1,0), M_wp(0,0))}; //pose of the extended node                
                
                //bounding box for the extended node + collision check
                const auto &obb = robot_->getOBB({node_extension.x, node_extension.y}, node_extension.theta);
                if(!robot_->isConfigurationFree(obb)) 
                {
                    collision = true;
                    break;
                }
                arc_end_points.poses.push_back(poseFromPose_(node_extension));
                if(norm(xr,yr) > Constants::Planner::max_res_) {break;}
                if(!ros::ok()) {break;}
            }
            if(!collision) {node_extensions_.push_back(node_extension);}
        }
        else
        {
            //delta is almost zero 
        }
    }
    arc_end_points_pub_.publish(arc_end_points);
    return node_extensions_; 
}

PRM::Pose_ PRM::rrt::getClosestPoseToGoal(const std::vector<Pose_> &poses, const Pose_ &goal_pose)
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

bool PRM::rrt::addPoseToTree(const Pose_ &pose, const rrt_nodePtr &parent, PoseToNodeMap &map)
{   
    rrt_nodePtr node = std::make_shared<rrt_node>();
    node->pose_ = pose;
    node->parent_ = parent;
    parent->children_.push_back(node);
    start_rtree_.insert(point_t{pose.x, pose.y}); //updating rtree
    map.insert({point_t{pose.x, pose.y}, node}); //updating map
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
        if (!req.start_runway.empty())
        {
            p.carveRunway(req.start_runway[0],req.start_runway[1],true);
        } 
        if (!req.goal_runway.empty())
        {
            p.carveRunway(req.goal_runway[0],req.goal_runway[1],false);
        }
        bool planned = plan(start, goal);
        ROS_INFO("planned: %d", planned);
        ros::Duration(2.0).sleep();
        p.repairPolygons();
        return planned;
    }
    return false;
}

bool PRM::rrt::plan(const geometry_msgs::PoseStamped &start, const geometry_msgs::PoseStamped &goal)
{   
    if(!goal_pose_set_ || !start_pose_set_ || !polygon_set_)
    {
        ROS_ERROR("start or goal pose or rrt_polygon is not set!");
        return false;
    }

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
    start_rtree_.insert(start_pt); //rtree
    start_rrt_map_.insert({start_pt, start_node}); //map

    auto start_time = std::chrono::high_resolution_clock::now();

    int max_iter = 10000;
    int iter_cnt = 0;

    while(ros::ok() && iter_cnt++ < max_iter)
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
            return false;
        }
        
        if(euclidDis(closest_node_->pose_, goal_pose) < Constants::Planner::max_res_)
        {
            ROS_WARN("Goal is within max_res_ ==> RRT converged!");
            publishRRTPath(closest_node_);
            return true; 
        }

        std::vector<Pose_> node_extensions_ = getNodeExtensions(closest_node_, Constants::Planner::max_res_);   
        //TO-DO ==> Add node-deletion logic
        if(node_extensions_.empty()) 
        {
            ROS_ERROR("No node extensions found! ==> CLOSEST NODE IS A DEAD-END ==> need to delete!"); 
            return false;
            break;
        }
        Pose_ closest_pose_ = getClosestPoseToGoal(node_extensions_, goal_pose);
        addPoseToTree(closest_pose_, closest_node_, start_rrt_map_);
    }

    auto end_time = std::chrono::system_clock::now();
    auto elapsed  = std::chrono::duration_cast<std::chrono::seconds>(end_time - start_time);
    
    ROS_WARN("================================================================================") ;
    ROS_WARN("RRT converged in %d seconds", elapsed.count());
    ROS_WARN("================================================================================") ;
    
    return false; 
}




