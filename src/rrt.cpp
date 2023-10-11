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

    //visualize_->publishT<geometry_msgs::PoseStamped>("test_start_pose", test_start_pose_, true);
    
    return; 
}

void PRM::rrt::goalPoseCb(geometry_msgs::PoseStampedConstPtr pose_)
{
    ROS_WARN("========== GOAL POSE RECEIVED =============="); 
    
    test_goal_pose_ = *pose_;

    goal_pose_set_ = true; 

   // visualize_->publishT<geometry_msgs::PoseStamped>("test_goal_pose", test_goal_pose_, true);

    if(!start_pose_set_)
    {
        ROS_ERROR("start_pose is not set!");
        return;
    }

    bool flag_ = plan(test_start_pose_, test_goal_pose_);

    if(flag_)
    {
        ROS_WARN("======= PLAN WAS FOUND ======");
        //Roadmap::success_cnt_++;
    }   
    

   
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


PRM::Pose_ PRM::rrt::sampleRandomPoint(const Polygon &polygon)
{   
    //ROS_INFO("==== Inside sampleRandomPoint ====");
    bool is_valid_ = bg::is_valid(polygon);

    if(!is_valid_)
    {
        ROS_ERROR("[sampleRandomPoint] => polygon is not valid!");
        return Pose_();
    }
    
    Box envelope;
    bg::envelope(polygon, envelope);

    std::random_device rd;
    std::mt19937 gen(rd());

    std::uniform_real_distribution<double> distX(bg::get<bg::min_corner, 0>(envelope), bg::get<bg::max_corner, 0>(envelope));
    std::uniform_real_distribution<double> distY(bg::get<bg::min_corner, 1>(envelope), bg::get<bg::max_corner, 1>(envelope));
    std::uniform_real_distribution<double> disTheta(0, 2 * M_PI);

    bool found_ = false; 

    int iter_limit_ = 100 * 100 * 100; 
    int num_iters_  = 0 ;


    while (ros::ok() && num_iters_++ < iter_limit_) 
    {
        //ROS_INFO("[sampleRandomPoint] => num_iters_: %d", num_iters_);
        point_t randomPoint(distX(gen), distY(gen));

        if (bg::within(randomPoint, polygon)) {
            float x = bg::get<0>(randomPoint);
            float y = bg::get<1>(randomPoint);
            float theta = disTheta(gen);

            auto obb_ = robot_->getOBB({x,y}, theta); 

            //std::cout << "before isConfigurationFree" << std::endl;
            if(robot_->isConfigurationFree(obb_))
            {   
                found_ = true; 
                return Pose_{x, y, theta}; 
            }
            //std::cout << "after isConfigurationFree" << std::endl;
        }
    }

    if(!found_)
    {
        ROS_ERROR("valid pose not found!");
        return Pose_();
    }
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

float PRM::rrt::getCost(const PRM::Pose_ &p1,  const PRM::Pose_ &p2) 
{
    
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
    const float y_dash_ = P_ab_(1,2);                                       // Δy in the frame of a
    

    float theta_dash_  = std::atan2(P_ab_(1,0), P_ab_(0,0));          // Δtheta in the frame of a
    
    if(theta_dash_ < 0)
    {
        theta_dash_ += 2 *  M_PI; 
    }
    
    const float r_ = Utils::getR(x_dash_, y_dash_);

    

    const float steering_dir_ = Utils::signDelta(x_dash_, y_dash_);
    const float theta_c_  = Utils::getThetaC(x_dash_, y_dash_, steering_dir_);

   // ROS_INFO("theta_dash_ => %f theta_c => %f", theta_dash_ * 180.f / M_PI, theta_c_ * 180.f / M_PI);   
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
            //implies reverse movement without turning
            ang_cost_ = 0 ; 
            dis_cost_ = Constants::Planner::w_rev_ * std::fabs(x_dash_);
        }
    }
    
    total_cost_ = dis_cost_ + ang_cost_;
    return total_cost_; 

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


    if(std::fabs(theta_dash_ - theta_c_) < Constants::Planner::theta_tol_)
    {
        return true;    
    }

    return false;

}


//generates a VALID random point inside the polygon 
            
PRM::Pose_ PRM::rrt::getNextPoint(const Polygon &polygon)
{
    bool found_  = false; 

    int iter_limit_ = 100 * 100 * 100;
    int num_iters_ = 0 ;

    while(ros::ok() && num_iters_++ < iter_limit_)
    {   
        //ROS_INFO("num_iters_: %d", num_iters_);
        Pose_ random_pose_ = sampleRandomPoint(polygon);   
        for(auto t: tree_)
        {   
            bool can_connect_ = canConnect(t->pose_, random_pose_); 
            if(can_connect_) { return random_pose_; }
        }
    }

    ROS_ERROR("valid pose not found!");
    return Pose_();
}

// check if possible to reach goal from the current tree
bool PRM::rrt::isGoalInVictinity(const PRM::Pose_ &pose)
{
    bool flag_ = false; 
    for(auto t: tree_)
    {
        Pose_ p = t->pose_; 
        if(canConnect(p, pose))
        {
            return flag_; 
        }
    }
    return flag_; 
}

//adds a new node to the tree
bool PRM::rrt::connectToTree(const PRM::Pose_ &pose)
{
    bool found = false; 
    rrt_nodePtr nearest_node_ = nullptr; 
    float min_cost_ = std::numeric_limits<float>::max();

    for(auto ptr_ : tree_)
    {
        float cost = getCost(ptr_->pose_, pose);
        if(cost < min_cost_)
        {
            min_cost_ = cost; 
            nearest_node_ = ptr_;
            found = true; 
        } 
    }

    if(!found) {
        ROS_ERROR("Unable to connect to tree => something is wrong!");
        return false; 
    }

    rrt_nodePtr new_node_ = std::make_shared<rrt_node>();
    new_node_->pose_ = pose;
    new_node_->parent_ = nearest_node_;
    new_node_->cost_ = nearest_node_->cost_ + min_cost_;
    
    nearest_node_->children_.push_back(new_node_);
    
    tree_.push_back(new_node_);

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

    rrt_nodePtr root_= std::make_shared<rrt_node>();
    root_->parent_ = nullptr; 
    root_->pose_ = Pose_{start_pose_};
    root_->cost_ = 0 ;
    tree_.push_back(root_);

    Polygon polygon_ = rrt_polygon_;

    int num_points = 0 ;
    while(ros::ok())
    {
        Pose_ nxt_pose_ = getNextPoint(polygon_);
        bool connect_flag_= connectToTree(nxt_pose_);
        ROS_INFO("tree_.size(): %d", tree_.size()); 
        if(isGoalInVictinity(goal_))
        {
            ROS_INFO("Goal in vicinity!");
            break;
        }
        num_points++;
        if(num_points % 10 == 0)
        {
            publishTree();
        }
    }

    return true; 
}




