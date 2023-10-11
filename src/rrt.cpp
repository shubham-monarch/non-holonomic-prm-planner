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
    rrt_tree_pub_ = nh_.advertise<geometry_msgs::PoseArray>("rrt_tree", 1, true);
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

    //polygon_set_ = true; 
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

void PRM::rrt::publishTree()
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

bool PRM::rrt::getCost(const PRM::Pose_ &p1,  const PRM::Pose_ &p2, float &cost) 
{
    if(!canConnect(p1, p2))
    {
        //ROS_ERROR("[getCost] can't connect the two points!");
        return false;
    }
    float dis_ = sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
        
    const float xa_ = p1.x, ya_ = p1.y;
    const float xb_ = p2.x, yb_ = p2.y;

    const Vec2f V_oa_{xa_, ya_};
    const Vec2f V_ob_{xb_, yb_};
    
    const float yaw_a_ = p1.theta; 
    const float yaw_b_ = p2.theta;

    const Mat3f &P_oa_ = (Utils::getHomogeneousTransformationMatrix(V_oa_, yaw_a_));
    const Mat3f &P_ob_ = (Utils::getHomogeneousTransformationMatrix(V_ob_, yaw_b_));
    const Mat3f &P_ao_ = P_oa_.inverse();
    const Mat3f &P_ab_ = P_ao_ * P_ob_;  //b in the frame of a
    
    const float x_dash_ = P_ab_(0,2);                                       // Δx in the frame of a 
    const float y_dash_ = P_ab_(1,2);          
                                 // Δy in the frame of a
    float theta_dash_  = std::atan2(P_ab_(1,0), P_ab_(0,0));          // Δtheta in the frame of a
    if(theta_dash_ < 0){ theta_dash_ += 2 *  M_PI; }
    
    const float r_ = Utils::getR(x_dash_, y_dash_);
    const float steering_dir_ = Utils::signDelta(x_dash_, y_dash_);
    const float theta_c_  = Utils::getThetaC(x_dash_, y_dash_, steering_dir_);
    float dis_cost_, ang_cost_; 
    float total_cost_; 

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
        if(x_dash_ >= 0)
        {   
            ang_cost_ = 0 ; 
            dis_cost_ = Constants::Planner::w_dis_ * x_dash_;
        } 
        else
        {
            ang_cost_ = 0 ; 
            dis_cost_ = Constants::Planner::w_rev_ * std::fabs(x_dash_);
        }
    }
    cost = dis_cost_ + ang_cost_;
    return true; 
}


bool PRM::rrt::canConnect(const PRM::Pose_ &p1,  const PRM::Pose_ &p2) 
{
    float dis_ = sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));

    if(dis_ > Constants::Planner::max_res_) {return false; }   
    if(dis_ < 0.001f) { return false; }      

    const float xa_ = p1.x, ya_ = p1.y;
    const float xb_ = p2.x, yb_ = p2.y;

    const Vec2f V_oa_{xa_, ya_};
    const Vec2f V_ob_{xb_, yb_};
    
    const float yaw_a_ = p1.theta; 
    const float yaw_b_ = p2.theta;

    const Mat3f &P_oa_ = (Utils::getHomogeneousTransformationMatrix(V_oa_, yaw_a_));
    const Mat3f &P_ob_ = (Utils::getHomogeneousTransformationMatrix(V_ob_, yaw_b_));
    const Mat3f &P_ao_ = P_oa_.inverse();        
    const Mat3f &P_ab_ = P_ao_ * P_ob_;  //b in the frame of a
    
    const float x_dash_ = P_ab_(0,2);                                       // Δx in the frame of a 
    const float y_dash_ = P_ab_(1,2);                                       // Δy in the frame of a
    
    if(x_dash_ < 0 && !Constants::Planner::can_reverse_) {return false; }
    
    float theta_dash_  = std::atan2(P_ab_(1,0), P_ab_(0,0));          // Δtheta in the frame of a
    if(theta_dash_ < 0) { theta_dash_ += 2 *  M_PI; }
    
    const float r_ = Utils::getR(x_dash_, y_dash_);
    if(r_ > 0.f && r_ < Constants::Vehicle::R_MIN_) { return false ;}
    
    const float steering_dir_ = Utils::signDelta(x_dash_, y_dash_);
    const float theta_c_  = Utils::getThetaC(x_dash_, y_dash_, steering_dir_);

    if(std::fabs(theta_dash_ - theta_c_) < Constants::Planner::theta_tol_) { return true; }
    
    return false;
}


//generates a VALID random point inside the polygon          
bool PRM::rrt::getNextPoint(const Polygon &polygon, PRM::Pose_ &nxt_pose_)
{
    bool found_  = false; 
    int iter_limit_ = 100 * 100 * 100;
    int num_iters_ = 0 ;
    while(ros::ok() && num_iters_++ < iter_limit_)
    {   
        Pose_ random_pose_; 
        bool can_sample_ = sampleRandomPoint(polygon, random_pose_);   
        if(!can_sample_) {continue;}
        for(auto t: tree_)
        {   
            bool can_connect_ = canConnect(t->pose_, random_pose_); 
            if(can_connect_) { 
                nxt_pose_ = random_pose_;
                return true;
            }
        }
    }
    return found_;
}

// check if possible to reach goal from the current tree
bool PRM::rrt::isGoalInVicinity(const PRM::Pose_ &pose)
{
    for(auto t: tree_)
    {
        Pose_ p = t->pose_; 
        if(canConnect(p, pose)) { return true; }   
    }
    return false; 
}

float PRM::rrt::euclidDis(const PRM::Pose_ &a, const PRM::Pose_ &b)
{
    return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2));
}

//adds a new node to the tree
bool PRM::rrt::connectToTree(const PRM::Pose_ &pose, rrt_nodePtr &new_node_)
{   
    //ROS_WARN("==== Inside connectToTree ====");
    bool found_ = false; 
    rrt_nodePtr nearest_node_ = nullptr; 
    float min_cost_ = std::numeric_limits<float>::max();
    int idx_ = 0;
    for(auto t : tree_)
    {   
        idx_++;
        bool can_connect_ = canConnect(t->pose_, pose);
        if(can_connect_)
        {
            found_ = true;
            float cost ;
            bool f_ = getCost(t->pose_, pose, cost); 
            if(f_ && (cost < min_cost_))
            {
                min_cost_ = cost; 
                nearest_node_ = t;
                //ROS_INFO("tree_idx: %d dis_from_nearest_node: %f", idx_, euclidDis(t->pose_, pose));               
            } 
        }
    }

    if(!found_) {
        ROS_ERROR("Unable to connect to tree => something is wrong!");
        return false; 
    }
    
    new_node_ = std::make_shared<rrt_node>();
    new_node_->pose_ = pose;
    new_node_->parent_ = nearest_node_;
    new_node_->cost_ = nearest_node_->cost_ + min_cost_;
    
    tree_.push_back(new_node_);
    return true; 
}


void PRM::rrt::printNode(const rrt_nodePtr &node)
{
    ROS_INFO("============================================");
    ROS_INFO("pose: (%f,%f,%f)", node->pose_.x, node->pose_.y, node->pose_.theta);
    ROS_INFO("cost: %f", node->cost_);
    ROS_INFO("parent: (%f,%f,%f)", node->parent_->pose_.x, node->parent_->pose_.y, node->parent_->pose_.theta);
    ROS_INFO("============================================");
}

// tries to correct the tree by checking if the new node can be connected to any other node in the tree
bool PRM::rrt::correctTree(rrt_nodePtr &new_node, const float sr)
{
    Pose_ pose_ = new_node->pose_;
    float node_cost_ = new_node->cost_;
    bool parent_updated_ = false; 

   // printNode(new_node);

    //attempt to update new_node parent
    for(auto t : tree_)
    {   
        if(t == new_node) {continue; }
        float dis_ = euclidDis(t->pose_, pose_);
        if(dis_ < sr)
        {
            bool can_connect_ = canConnect(t->pose_, pose_);
            if(can_connect_)
            {
                float curr_cost;
                bool f_ = getCost(t->pose_, pose_, curr_cost);

                if(f_ && (t->cost_ + curr_cost < node_cost_))
                {
                    //node parent need to be updated
                    parent_updated_ = true; 
                    new_node->parent_ = t;
                    new_node->cost_ = t->cost_ + curr_cost;
                }
            }
        }
    }
    
    /*if(parent_updated_)
    {
        ROS_WARN("======= parent updated ======="); 
        printNode(new_node); 
    }*/

    //attempt to update the nodes in the vicinity of the new node
    for(auto t: tree_)
    {
        if(t == new_node) {continue;}
        float dis = euclidDis(pose_, pose_);
        if(dis < sr)
        {
            float curr_cost;
            if(getCost(pose_, t->pose_, curr_cost))
            {
                if(node_cost_ + curr_cost < t->cost_)
                {
                    //t parent need to be updated
                    t->parent_ = new_node;
                    t->cost_ = node_cost_ + curr_cost;
                } 
            };
        }
    }
    return true; 
}

bool PRM::rrt::plan(const geometry_msgs::PoseStamped &start_pose_, const geometry_msgs::PoseStamped &goal_pose_)
{   
    if(!goal_pose_set_ || !start_pose_set_ || !polygon_set_)
    {
        ROS_ERROR("start or goal pose is not set!");
        return false;
    }

    CollisionDetectionPolygon &p = robot_->getCollisionPolyRef(); 
    bool f_ = p.selectCurrentIndex(Point_t(start_pose_.pose.position.x, start_pose_.pose.position.y), \
                                            Point_t(goal_pose_.pose.position.x, goal_pose_.pose.position.y));

    if(!f_) 
    {
        ROS_ERROR("unable to select current index!");
        return false;
    }

    Pose_ goal_{goal_pose_};
    Pose_ start_{start_pose_};

    rrt_nodePtr root_= std::make_shared<rrt_node>();
    root_->parent_ = nullptr; 
    root_->pose_ = Pose_{start_pose_};
    root_->cost_ = 0 ;
    tree_.push_back(root_);

    Polygon polygon_ = rrt_polygon_;

    ROS_INFO("max_res_: %f", Constants::Planner::max_res_);

    int num_points = 0 ;

    auto start_time = std::chrono::high_resolution_clock::now();
    
    while(ros::ok())
    {
        Pose_ nxt_pose_; 
        bool nxt_pose_found_ = getNextPoint(polygon_, nxt_pose_);
        
        if(!nxt_pose_found_)
        {
            ROS_ERROR(" === Can't expand RRT TREE =="); 
            return false;
        }

        rrt_nodePtr nxt_node_ = nullptr;
        bool connect_flag_= connectToTree(nxt_pose_, nxt_node_);
        if(!connect_flag_)
        {
            ROS_ERROR("Unable to connect next point to tree ==> Something is wrong!");
            return false;
        }

        bool correct_flag_= correctTree(nxt_node_, Constants::Planner::sr_);

        if(isGoalInVicinity(start_))
        {
            ROS_INFO("Goal in vicinity!");
            //ros::Duration(10.0).sleep();
            break;
        }

        //num_points++;
        //ros::Duration(1.0).sleep();
        if(num_points % 100 == 0)
        {
              publishTree();
              ros::Duration(1.0).sleep();
              ros::spinOnce();
        }

        //ros::Duration(0.1).sleep();
    }

    auto end_time = std::chrono::system_clock::now();
    auto elapsed  = std::chrono::duration_cast<std::chrono::seconds>(end_time - start_time);
    
    ROS_WARN("================================================================================") ;
    ROS_WARN("RRT Trre with %d nodes took %ld seconds!", (int)tree_.size(), elapsed.count());
    ROS_WARN("================================================================================") ;

    return true; 
}




