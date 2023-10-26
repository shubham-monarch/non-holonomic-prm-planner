#include <non-holonomic-prm-planner/rrt.h>
#include <non-holonomic-prm-planner/path_generator.h>
#include <non-holonomic-prm-planner/helpers.h>

#include <random>
#include <cmath>

#include <geometry_msgs/PoseArray.h>

#include <boost/geometry/geometries/ring.hpp>
#include <boost/geometry/algorithms/append.hpp>
#include <boost/geometry/algorithms/for_each.hpp>
#include <boost/geometry/geometries/linestring.hpp>

extern std::shared_ptr<PRM::RobotModel> robot_;

PRM::rrt::rrt():start_pose_set_{false}, goal_pose_set_{false}, polygon_set_{false}, geofence_set_{false}
{
    //polygon_ = Polygon();
    //polygon_.outer().push_back(point_t(0, 0));
    //polygon_.outer().push_back(point_t(0, 1));
    //polygon_.outer().push_back(point_t(1, 1));
    //polygon_.outer().push_back(point_t(1, 0)

    //reset();

    start_pose_sub_ = nh_.subscribe("/initialpose", 1, &rrt::initialPoseCb, this);
    goal_pose_sub_ = nh_.subscribe("/goal", 1, &rrt::goalPoseCb, this);
    geofence_sub_ = nh_.subscribe("/geofence", 1, &rrt::geofenceCb, this);

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
    poly_centroid_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("poly_centroid", 1, true);

    st_tree_pub_ = nh_.advertise<geometry_msgs::PoseArray>("st_tree", 1, true); 
    go_tree_pub_ = nh_.advertise<geometry_msgs::PoseArray>("go_tree", 1, true);

    start_tree_pub_ = nh_.advertise<geometry_msgs::PoseArray>("start_tree", 1, true);
    goal_tree_pub_ = nh_.advertise<geometry_msgs::PoseArray>("goal_tree", 1, true);

    ros_path_pub_ = nh_.advertise<nav_msgs::Path>("ros_path", 1, true);
    line_arr_pub_ = nh_.advertise<geometry_msgs::PoseArray>("line_arr", 1, true);
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
        
        //ROS_INFO("polygon is valid!");
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

void PRM::rrt::geofenceCb(geometry_msgs::PolygonStampedConstPtr msg)
{
    if(geofence_set_) {return; }
    ROS_WARN("========== GEOFENCE POLYGON RECEIVED ==============");  
    Polygon polygon_ = getPolygonFromPolygonMsg(*msg);
    //ROS_INFO("geofence_polygon.size(): %d", polygon_.outer().size());
    if(polygon_.outer().size() > 0)
    {
        geofence_polygon = polygon_;
        geofence_set_ = true; 
    }
    else
    {
        ROS_ERROR("[geofenceCb] => Geofence is not valid!");
        return;
    }
}

void PRM::rrt::reset()
{
    goal_pose_set_ = false; 
    start_pose_set_ = false;
    polygon_set_ = false;

    //st_rtree_ = std::make_shared<RTree>();
    //go_rtree_ = std::make_shared<RTree>();
    curr_rtree_ = std::make_shared<RTree>();
    
    //st_pose_to_node_map_ = std::make_shared<PoseToNodeMap>();
    //go_pose_to_node_map_ = std::make_shared<PoseToNodeMap>();
    curr_pose_to_node_map_ = std::make_shared<PoseToNodeMap>();
    
    st_dmap_ = std::make_shared<DMap>();
    go_dmap_ = std::make_shared<DMap>();
    curr_dmap_ = std::make_shared<DMap>();

    rrt_tree_.poses.clear();
    combined_tree_.clear();

    goal_tree_pts_.poses.clear(); 
    start_tree_pts_.poses.clear();
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

PRM::Pose_ PRM::rrt::sampleRandomPoint(const geometry_msgs::PoseStamped &centroid)
{   
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> distX(-10, 10);
    std::uniform_real_distribution<double> distY(-10, 10);
    std::uniform_real_distribution<double> disTheta(0, 2 * M_PI);
    
    int iter_limit_ = 100 * 100 * 100; 
    int num_iters_  = 0 ;

    geometry_msgs::PoseStamped centroid_pose_;
    centroid_pose_.header.frame_id = "map";
    centroid_pose_.header.stamp = ros::Time::now();

    float cx = centroid.pose.position.x;  
    float cy = centroid.pose.position.y; 

    //CollisionDetectionPolygon &p = robot_->getCollisionPolyRef();
    while (ros::ok() && num_iters_++ < iter_limit_) 
    {
        float x  = cx + distX(gen), y = cy + distY(gen) , theta = disTheta(gen);
        if(robot_->isConfigurationFree(robot_->getOBB({x, y}, theta)))
        {
            Pose_ pose_ = Pose_(x, y, theta);
            centroid_pose_.pose.position.x = x;  
            centroid_pose_.pose.position.y = y; 
            centroid_pose_.pose.orientation = tf::createQuaternionMsgFromYaw(theta);
            poly_centroid_pub_.publish(centroid_pose_);
            return pose_;
        }
    }
    return Pose_(); 
}


bool PRM::rrt::sampleRandomPolygonPoint(const Polygon &polygon, PRM::Pose_ &pose)
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
    std::uniform_real_distribution<double> distX(-10, 10);
    std::uniform_real_distribution<double> distY(-10, 10);
    std::uniform_real_distribution<double> disTheta(0, 2 * M_PI);
    
    int iter_limit_ = 100 * 100 * 100; 
    int num_iters_  = 0 ;

    point_t centroid; 
    bg::centroid(polygon, centroid);

    geometry_msgs::PoseStamped centroid_pose_; 
    centroid_pose_.header.frame_id = "map";
    centroid_pose_.header.stamp = ros::Time::now();
    

    float cx = centroid.x(); 
    float cy = centroid.y(); 

    //CollisionDetectionPolygon &p = robot_->getCollisionPolyRef();
    while (ros::ok() && num_iters_++ < iter_limit_) 
    {
        point_t randomPoint(cx + distX(gen), cy + distY(gen));
        if (bg::within(randomPoint, polygon)) {
           
            bool flag = robot_->getCollisionPolyRef().isConfigurationFree(randomPoint.x(), randomPoint.y());
            if(flag)
            {
                pose = Pose_(randomPoint.x(), randomPoint.y(), disTheta(gen));
                centroid_pose_.pose.position.x = randomPoint.x(); 
                centroid_pose_.pose.position.y = randomPoint.y(); 
                centroid_pose_.pose.orientation = tf::createQuaternionMsgFromYaw(0);
                poly_centroid_pub_.publish(centroid_pose_);
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
        
        geometry_msgs::PoseStamped centroid_pose;
        bool centroid_found_ = estimateSamplingCentroid(start, goal, centroid_pose);    
        if(!centroid_found_) {
            ROS_ERROR("Polygon centroid not found!");
            return false; 
        }
        bool planned = biDirectionalPlan(start, goal);
        
        ROS_DEBUG("======================================================================") ;
        ROS_DEBUG("planned: %d", planned);
        ROS_DEBUG("======================================================================") ;
        
        ros::Duration(7.0).sleep();    
        p.repairPolygons();

        return planned;
    }
    return false;
}

bool PRM::rrt::getPoseProjectionOnGeofence(const geometry_msgs::PoseStamped &pose, const bool fwd, geometry_msgs::PoseStamped &projected_pose )
{
    float x = pose.pose.position.x, y = pose.pose.position.y , theta = tf::getYaw(pose.pose.orientation); 
    if(!fwd) {theta = theta + M_PI ;}
    int num_iter = 0 , mx_iter = 1000 ;
    float step_sz = 1.f;
    bool found = false; 
    bool prev_is_free = false, curr_is_free = false; 
    while(ros::ok() && (num_iter++ < mx_iter))
    {
        x = x + step_sz * cos(theta);
        y = y + step_sz * sin(theta);
        curr_is_free = robot_->isConfigurationFree(robot_->getOBB({x,y}, theta)); 
        if(!curr_is_free && prev_is_free)
        {
            found = true; 
            projected_pose.header.frame_id = "map";
            projected_pose.header.stamp = ros::Time::now();
            projected_pose.pose.position.x = x; 
            projected_pose.pose.position.y = y;
            projected_pose.pose.orientation = tf::createQuaternionMsgFromYaw(theta);
            break;
        }
        prev_is_free = curr_is_free;
    }
    if(!found) {return false; }
    return true; 
}

void PRM::rrt::publishPoint(const point_t pt, ros::Publisher &pub)
{
    geometry_msgs::PoseStamped pose_; 
    pose_.header.frame_id = "map"; 
    pose_.header.stamp = ros::Time::now();
    pose_.pose.position.x = pt.x(); 
    pose_.pose.position.y = pt.y(); 
    pub.publish(pose_);
}

geometry_msgs::Pose PRM::rrt::poseFromPt(const point_t pt)
{
    geometry_msgs::Pose pose_; 
    pose_.position.x = pt.x(); 
    pose_.position.y = pt.y();
    pose_.orientation = tf::createQuaternionMsgFromYaw(0);
    return pose_;
}

geometry_msgs::PoseStamped PRM::rrt::poseStampedFromPt(const point_t pt)
{
    geometry_msgs::PoseStamped pose_; 
    pose_.header.frame_id = "map";
    pose_.header.stamp = ros::Time::now();
    pose_.pose.position.x = pt.x();
    pose_.pose.position.y = pt.y();
    pose_.pose.orientation = tf::createQuaternionMsgFromYaw(0);
    return pose_;
}

bool PRM::rrt::estimateSamplingCentroid(    const geometry_msgs::PoseStamped &start_pose_, const geometry_msgs::PoseStamped &goal_pose_, \
                                            geometry_msgs::PoseStamped &centroid_pose_)
{   
    while(!geofence_set_ && ros::ok())
    {
        ROS_DEBUG("waiting for geofence_set to be true!");
        ros::spinOnce();
        ros::Duration(0.1).sleep();
    }
    typedef bg::model::linestring<point_t> Linestring;
    Linestring line; 
    float x1 = start_pose_.pose.position.x, x2 = goal_pose_.pose.position.x;
    float y1 = start_pose_.pose.position.y, y2 = goal_pose_.pose.position.y;
    float theta = atan2(y2 - y1, x2 - x1) + M_PI/ 2.f; 
    point_t p1 = point_t{(x1 + x2)/2.f, (y1 + y2)/2.f} ;
    point_t p2 = point_t{p1.x() + 10 * cos(theta), p1.y() + 10 * sin(theta)};
    point_t p3 = point_t{p1.x() - 10 * cos(theta), p1.y() - 10 * sin(theta)};
    
    /*geometry_msgs::PoseArray line_arr_; 
    line_arr_.header.frame_id = "map";
    line_arr_.header.stamp = ros::Time::now();
    line_arr_.poses.push_back(poseFromPt(p2));
    line_arr_.poses.push_back(poseFromPt(p3));
    line_arr_pub_.publish(line_arr_);
    */

    bg::append(line, p3);
    bg::append(line,p2);
    std::vector<point_t> points; 
    bg::intersection(line, geofence_polygon, points); 

    if(points.size() > 1)
    {
        ROS_ERROR("Line segment intesection with geofence_polygon at multiple points!");
        return false; 
    }   

    point_t centroid_pt = point_t{(0.7 * points[0].x() + 0.3 * p1.x()) , 0.7 * points[0].y() + 0.3 * p1.y()};
    centroid_pose_ = poseStampedFromPt(centroid_pt);
    centroid_pose_.pose.orientation = tf::createQuaternionMsgFromYaw(theta - M_PI/2.f);
    //centroid_pose_ = poseFromPt(p3);
    poly_centroid_pub_.publish(centroid_pose_);
    //poly_centroid_pub_.publish(start_pose_);
    poly_centroid_pose_ = centroid_pose_;

   
   return true; 
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

bool PRM::rrt::canConnectToOtherTree(const Pose_ &pose, const rrt_containerPtr &other_container_, bool fwd)
{
    point_t pt{pose.x, pose.y}; 
    bool found_; 
    
    const RTreePtr &rtree_ = other_container_->rtree_; 
    const PoseToNodeMapPtr &pose_to_node_map_ = other_container_->pose_to_node_map_;

    std::vector<point_t> closest_points_;
    rtree_->query(bgi::nearest(point_t{pose.x, pose.y}, 1), std::back_inserter(closest_points_));
    if(closest_points_.size() == 0) { 
        ROS_ERROR("No closest point found in rtree!");
        return false; 
    }

    point_t other_pt_ = closest_points_[0];
    //Pose_ other_pose_; 
    auto itr = pose_to_node_map_->find(other_pt_);
    if(itr == curr_pose_to_node_map_->end()) { 
        ROS_ERROR("No closest node found in pose_to_node_map! ==> Something is wrong!");
        return false; 
    }
    const Pose_ other_pose_ = itr->second->pose_;

    return(canConnect(pose, other_pose_, fwd));    
}


void PRM::rrt::publishROSPath(const std::vector<rrt_nodePtr> &tree)
{

    std::vector<PRM::Node3d> path_; 
    for(auto t : tree)
    {

        Node3d node_{t->pose_.x, t->pose_.y, t->pose_.theta};
        path_.push_back(node_);
    }

    nav_msgs::Path ros_path_ = nav_msgs::Path(); 
    ros_path_.header.frame_id = "map"; 
    ros_path_.header.stamp = ros::Time::now(); 
    ros_path_pub_.publish(ros_path_);

    ros_path_ = Utils::generateROSPath(path_);
   ros_path_pub_.publish(ros_path_);

}

void PRM::rrt::publishStartAndGoalTree()
{
    ROS_INFO("Inside publishStartAndGoalTree()!");
    geometry_msgs::PoseArray st_tree_arr_, go_tree_arr_;
    st_tree_arr_.header.frame_id = "map";
    st_tree_arr_.header.stamp = ros::Time::now();

    
    std::vector<rrt_nodePtr> v; 

    while(st_found_node_ != nullptr) 
    {
        v.push_back(st_found_node_);
        st_tree_arr_.poses.push_back(poseFromPose_(st_found_node_->pose_));
        st_found_node_ = st_found_node_->parent_;
    }
    st_tree_pub_.publish(st_tree_arr_); 

    std::reverse(v.begin(), v.end()); 
    for(auto t: v) {combined_tree_.push_back(t) ;}
    v.clear();
    go_tree_arr_.header.frame_id = "map";
    go_tree_arr_.header.stamp = ros::Time::now();
    while(go_found_node_ != nullptr)
    {   
        v.push_back(go_found_node_);
        go_tree_arr_.poses.push_back(poseFromPose_(go_found_node_->pose_));
        go_found_node_ = go_found_node_->parent_;
    }
    go_tree_pub_.publish(go_tree_arr_);
    for(auto t: v) {combined_tree_.push_back(t); }
    publishROSPath(combined_tree_);
}


bool PRM::rrt::biDirectionalPlan(const geometry_msgs::PoseStamped &start, const geometry_msgs::PoseStamped &goal)
{   
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
    
    rrt_containerPtr st_container_ = std::make_shared<rrt_container>(); 
    st_container_->rtree_->insert(start_pt); //rtree
    st_container_->pose_to_node_map_->insert({start_pt, start_node}); //map
    // =======================================================================================
    
    // =======================================================================================
    // ============================ RRT FROM GOAL POSE =======================================
    // =======================================================================================
    rrt_nodePtr go_node = std::make_shared<rrt_node>();  //first node of start rrt
    go_node->parent_ = nullptr; 
    go_node->pose_ = goal_pose;
    go_node->cost_ = 0 ;  

    rrt_containerPtr go_container_ = std::make_shared<rrt_container>(); 
    go_container_->rtree_->insert(goal_pt); //rtree
    go_container_->pose_to_node_map_->insert({goal_pt, go_node}); //map
    //=========================================================================================

    // === setting curr vars ======
    //curr_container_ = go_container_; 
    //setRtree(curr_container_->rtree_);
    //setPoseToNodeMap(curr_container_->pose_to_node_map_);
    //setDMap(curr_container_->dmap_);
    // ============================
    //setRRTContainer(go_container_);
    
    rrt_containerPtr c1_, c2_; //containers for rrts from start and goal poses

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

        if(iter_cnt % 2)
        {
            //continue;
            //c1_ = st_container_; 
            //c2_= go_container_;
            setContainer(st_container_); 
        }
        else
        {
            //c1_ = go_container_;
            //c2_ = st_container_;
            //continue;
            setContainer(go_container_);
        }

        //setContainer(c1_);

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

        Pose_ nxt_pose = sampleRandomPoint(poly_centroid_pose_);   
        
        rrt_nodePtr closest_node_; 
        bool found = getNearestNodeToSampledPoint(nxt_pose, closest_node_);
        if(!found) {
            ROS_ERROR("No closest node found! ==> Breaking while loop!"); 
            return false;
        }
        
        //std::vector<Pose_> node_extensions_ = getNodeExtensions(closest_node_, Constants::Planner::max_res_, (iter_cnt % 2 ? true : false));   
        std::vector<Pose_> node_extensions_ = getNodeExtensions(closest_node_, Constants::Planner::max_res_/2.f, (iter_cnt % 2 ? true : false));   
        //TO-DO ==> Add node-deletion logic
        if(node_extensions_.empty()) 
        {   
            //publishRRTPath(closest_node_);
            ROS_ERROR("No node extensions found! ==> CLOSEST NODE IS A DEAD-END ==> deleting!"); 
            deleteNode(closest_node_->pose_);
            continue;
        }
        //Pose_ closest_pose_ = getClosestNodeExtensionToGoal(node_extensions_, (iter_cnt % 2 ? goal_pose: start_pose));
        Pose_ closest_pose_ = getClosestNodeExtensionToGoal(node_extensions_, nxt_pose);
        addPoseToTree(closest_pose_, closest_node_);


        if(iter_cnt %2 ==0 )
        {
            start_tree_pts_.header.frame_id = "map";
            start_tree_pts_.header.stamp = ros::Time::now();
            start_tree_pts_.poses.push_back(poseFromPose_(closest_pose_));    
            start_tree_pub_.publish(start_tree_pts_);
        }
        else
        {
            goal_tree_pts_.header.frame_id = "map";
            goal_tree_pts_.header.stamp = ros::Time::now();
            goal_tree_pts_.poses.push_back(poseFromPose_(closest_pose_));     
            goal_tree_pub_.publish(goal_tree_pts_);
        }
        
        bool can_connect_ = false; 
        if(iter_cnt % 2)
        {
            can_connect_ = canConnectToOtherTree(closest_pose_, go_container_, true);
        }
        else
        {
            can_connect_ = canConnectToOtherTree(closest_pose_, st_container_, false);
        }

        if(can_connect_)
        {   

            ROS_DEBUG("Two RRTs are converging!"); 
            std::vector<point_t> v;
            point_t st_pt, go_pt;
            
            if(iter_cnt % 2) 
            {   
                point_t st_pt{closest_pose_.x, closest_pose_.y};
                auto itr = st_container_->pose_to_node_map_->find(st_pt);
                st_found_node_= itr->second;
                
                go_container_->rtree_->query(bgi::nearest(st_pt, 1), std::back_inserter(v));
                go_pt = v[0];
                itr = go_container_->pose_to_node_map_->find(go_pt);
                go_found_node_ = itr->second;    

            }
            else
            {
                point_t go_pt{closest_pose_.x, closest_pose_.y};
                auto itr = go_container_->pose_to_node_map_->find(go_pt);
                go_found_node_= itr->second;

                st_container_->rtree_->query(bgi::nearest(go_pt, 1), std::back_inserter(v));
                st_pt = v[0];
                itr = st_container_->pose_to_node_map_->find(st_pt);
                st_found_node_ = itr->second;
                
            }
            publishStartAndGoalTree();
            goal_reached = true; 
            break; 
        }

    }

    auto end_time = std::chrono::system_clock::now();
    auto elapsed  = std::chrono::duration_cast<std::chrono::seconds>(end_time - start_time);
    
    ROS_DEBUG("================================================================================") ;
    ROS_DEBUG("RRT converged in %d seconds", elapsed.count());
    ROS_DEBUG("================================================================================") ;
    
    return goal_reached; 
}



