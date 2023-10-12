#include <non-holonomic-prm-planner/rrt.h>
#include <non-holonomic-prm-planner/path_generator.h>

#include <random>

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
}

void PRM::rrt::initialPoseCb(geometry_msgs::PoseWithCovarianceStampedConstPtr pose_)
{
    ROS_WARN("========== START POSE RECEIVED =============="); 
    test_start_pose_.header.frame_id= "map" ; 
    test_start_pose_.header.stamp = ros::Time::now(); 
    test_start_pose_.pose.position.x = pose_->pose.pose.position.x; 
    test_start_pose_.pose.position.y = pose_->pose.pose.position.y;
    test_start_pose_.pose.orientation = pose_->pose.pose.orientation;   
    start_pose_set_ = true; 
    start_pose_pub_.publish(test_start_pose_);
    return; 
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
    bool flag_ = plan(test_start_pose_, test_goal_pose_);
    if(flag_) { ROS_WARN("======= PLAN WAS FOUND ======"); }        
    return;
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

float PRM::rrt::euclidDis(const PRM::Pose_ &a, const PRM::Pose_ &b)
{
    return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2));
}

void PRM::rrt::printNode(const rrt_nodePtr &node)
{
    ROS_INFO("============================================");
    ROS_INFO("pose: (%f,%f,%f)", node->pose_.x, node->pose_.y, node->pose_.theta);
    ROS_INFO("cost: %f", node->cost_);
    ROS_INFO("parent: (%f,%f,%f)", node->parent_->pose_.x, node->parent_->pose_.y, node->parent_->pose_.theta);
    ROS_INFO("============================================");
}

bool PRM::rrt::extendTree(const std::vector<PRM::rrt_nodePtr> &tree, const PRM::Pose_ &pose_)
{    
}

bool PRM::rrt::plan(const geometry_msgs::PoseStamped &start_pose_, const geometry_msgs::PoseStamped &goal_pose_)
{   
    if(!goal_pose_set_ || !start_pose_set_ || !polygon_set_)
    {
        ROS_ERROR("start or goal pose or rrt_polygon is not set!");
        return false;
    }

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

    rrt_nodePtr start_root_= std::make_shared<rrt_node>();  //first node of start rrt
    start_root_->parent_ = nullptr; 
    start_root_->pose_ = Pose_{start_pose_};
    start_root_->cost_ = 0 ;
    start_rrt_.push_back(start_root_);  //updating start rrt
    start_rtree_.insert(point_t{start_.x, start_.y}); //updating start rtree
    //start_rrt_map_[start_pt] = start_root_; //updating start rrt map 
    
    rrt_nodePtr goal_root_ = std::make_shared<rrt_node>();  //first node of goal rrt
    goal_root_->parent_ = nullptr; 
    goal_root_->pose_ = Pose_{start_pose_};
    goal_root_->cost_ = 0 ;
    goal_rrt_.push_back(goal_root_);  //updating start rrt
    goal_rtree_.insert(point_t{goal_.x, goal_.y}); //updating start rtree
    //goal_rrt_map_[goal_pt] = goal_root_; //updating start rrt map 
    
    ROS_INFO("max_res_: %f", Constants::Planner::max_res_);

    int num_points = 0 ;
    auto start_time = std::chrono::high_resolution_clock::now();
    Polygon polygon_ = rrt_polygon_;
    int tree_idx = 0 ;

    while(ros::ok())
    {
        Pose_ nxt_pose_; 
        bool nxt_pose_found_ = sampleRandomPoint(polygon_, nxt_pose_);
        
        if(!nxt_pose_found_)
        {
            ROS_ERROR(" === Could not sample a random point ==> Trying again === "); 
            return false;
        }

        //update start tree
        if(tree_idx % 2)
        {
            extendTree(start_rrt_, nxt_pose_);   
        }
        else //update goal tree
        {

        }

        tree_idx +=1;
        tree_idx %=2; 
    }

    auto end_time = std::chrono::system_clock::now();
    auto elapsed  = std::chrono::duration_cast<std::chrono::seconds>(end_time - start_time);
    
    ROS_WARN("================================================================================") ;
    ROS_WARN("RRT converged in %d seconds", elapsed.count());
    ROS_WARN("================================================================================") ;
    
    return true; 
}




