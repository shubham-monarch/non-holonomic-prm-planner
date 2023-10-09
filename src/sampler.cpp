#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>

#include <random>

#include <non-holonomic-prm-planner/sampler.h>
#include <non-holonomic-prm-planner/roadmap.h>

#include <ros/topic.h>
#include <chrono>


//extern 
extern std::shared_ptr<PRM::RobotModel> robot_;
extern std::shared_ptr<PRM::Visualize> visualize_;


PRM::Sampler::Sampler(const std::string topic): sampled_points_topic_(topic)
{   
    //std::cout << "Sampler constructor called!" << std::endl;
    ROS_DEBUG("Inside sampler constructor!");
    //sampled_points_sub_ = nh_.subscribe(sampled_points_topic_, 1, &PRM::Sampler::sampledPointsCallback, this);

    //ros::topic::waitForMessage(sampled_points_topic_, nh_);

    //ros::topic::waitForMessage(sampled_points_topic_, nh_)

    //ros::topic::waitForMessage<geometry_msgs::PoseArray>(sampled_points_topic_);

    //ros::topic::waitForMessage    
    //samplePointsForRowTransition(geometry_msgs::PoseStamped(), geome);

    lowest_pose_pub = nh_.advertise<geometry_msgs::PoseStamped>("centre_pose", 1, true);
    current_polygon_pub = nh_.advertise<geometry_msgs::PolygonStamped>("current_polygon", 1, true);
}

geometry_msgs::PoseStamped shiftPose(const geometry_msgs::PoseStamped &pose_, float dis_, bool fwd)
{   

    float yaw_ = tf::getYaw(pose_.pose.orientation);

    float x, y; 

    float dir_ = (fwd ? 1: -1);

    x = pose_.pose.position.x + dir_ * dis_ *  cos(yaw_); 
    y = pose_.pose.position.y + dir_ * dis_ * sin(yaw_);

    geometry_msgs::PoseStamped shifted_pose_; 
    shifted_pose_.header.frame_id = "map"; 
    shifted_pose_.header.stamp = ros::Time::now(); 
    shifted_pose_.pose.position.x = x;
    shifted_pose_.pose.position.y = y;
    shifted_pose_.pose.orientation = pose_.pose.orientation;
    
    return shifted_pose_;

}




void PRM::Sampler::getRunwayPolygon(const geometry_msgs::PoseStamped &start_pose_, const geometry_msgs::PoseStamped &goal_pose_, \
                                const bool start_)
{

    Point goal_pt = Point{goal_pose_.pose.position.x, goal_pose_.pose.position.y};
    Point start_pt = Point{start_pose_.pose.position.x, start_pose_.pose.position.y};

    Point pt_ = (start_ ?  Point{start_pose_} : Point{goal_pose_});

    float theta_parallel = std::atan2(goal_pt.y - start_pt.y, goal_pt.x - start_pt.x);
    float theta_perependicular_ = tf::getYaw(start_pose_.pose.orientation);

    float dir_ = (start_ ? 1.f : -1.f); 

    ROS_INFO("dir_: %f", dir_);

    Point v1 = Point{pt_.x - 2.f * std::cos(theta_parallel), pt_.y - 2.f * std::sin(theta_parallel)}; 
    Point v2 = Point{v1.x +   10 * std::cos(theta_perependicular_), v1.y + 10 *  std::sin(theta_perependicular_)};
    Point v3 = Point{v2.x + 4.f * std::cos(theta_parallel), v2.y + 4.f * std::sin(theta_parallel)}; 
    Point v4 = Point{v3.x -  10 * std::cos(theta_perependicular_), v3.y -  10 * std::sin(theta_perependicular_)};
    
    std::vector<Point> vertices_{v1, v2, v3, v4};

    geometry_msgs::PolygonStamped poly_msg;
    poly_msg.header.frame_id = "map";
    poly_msg.header.stamp = ros::Time::now();

    for(auto t: vertices_)
    {
        geometry_msgs::Point32 pt;
        pt.x = t.x;
        pt.y = t.y;
        pt.z = 0;
        poly_msg.polygon.points.push_back(pt);
    }

   // visualize_->publishT<geometry_msgs::PolygonStamped>("current_polygon", poly_msg);
    current_polygon_pub.publish(poly_msg);

    return ;
}   


std::vector<PRM::Node2d> PRM::Sampler::gaussianSampleBetweenWhiteAndGreenpolygons(const geometry_msgs::PoseStamped &start_pose_, \
                                                        const geometry_msgs::PoseStamped &goal_pose_, 
                                                        const int num_points)
{

    //== 'section' => horizontal section between the start and the goal row 

    //ROS_WARN("=== Gaussian Sample! ====");

    geometry_msgs::PoseStamped section_centre_; 

    //std::cout << "start_pose: (" << start_pose_.pose.position.x << "," << start_pose_.pose.position.y << ")" << std::endl;
    //std::cout << "goal_pose: (" << goal_pose_.pose.position.x << "," << goal_pose_.pose.position.y << ")" << std::endl;
    
   // bool flag_ = getPolygonCenter(start_pose_, goal_pose_, section_centre_);

    geometry_msgs::PoseStamped l1_, l2_; //lowest pose for start and goal poses 
    bool f1_, f2_; 
    
    f1_ = getLowestPoint(start_pose_, goal_pose_, start_pose_, l1_);
    f2_ = getLowestPoint(start_pose_, goal_pose_, goal_pose_, l2_);

    if(!f1_ || !f2_)
    {
        ROS_ERROR("unable to find lowest point for start OR goal pose!");
        return std::vector<Node2d>();
    }

    Point st_pt{start_pose_.pose.position.x, start_pose_.pose.position.y}; 
    Point go_pt{goal_pose_.pose.position.x, goal_pose_.pose.position.y};

    Point l1_pt{l1_.pose.position.x, l1_.pose.position.y}; 
    Point l2_pt{l2_.pose.position.x, l2_.pose.position.y};

    float x_mx, x_mn; 
    float y_mx, y_mn; 

    std::vector<float> vx{st_pt.x, go_pt.x , l1_pt.x, l2_pt.x}; 
    std::vector<float> vy{st_pt.y, go_pt.y, l1_pt.y, l2_pt.y};

    x_mx = *std::max_element(vx.begin()  , vx.end());
    x_mn = *std::min_element(vx.begin(), vx.end()); 

    y_mx = *std::max_element(vy.begin()  , vy.end());
    y_mn = *std::min_element(vy.begin(), vy.end()); 

    std::random_device rd;
    std::mt19937 gen(rd());
    
    //std::uniform_real_distribution<float > x_dis(p_centre.x - 0.5 * section_width * cos(theta), p_centre.x +  0.5 * section_width * cos(theta)); 
    //std::uniform_real_distribution<float > y_dis(p_centre.y - section_len * sin(theta), p_centre.y +  section_len * sin(theta)); 

    std::uniform_real_distribution<float > x_dis(x_mn - 5, x_mx + 5); 
    std::uniform_real_distribution<float > y_dis(y_mn - 5, y_mx + 5); 
    std::uniform_real_distribution<float > theta_dis(0, 2 * M_PI); //theta distribution
    

    auto start_time = std::chrono::system_clock::now();
    // Perform some time-consuming operation
    int cnt= 0 ; 

    float sigma = 1.f; 
    std::normal_distribution<> normal_dis(0.f, sigma);


    sampled_points_.clear();

    CollisionDetectionPolygon &poly_ = robot_->getCollisionPolyRef();

    while(ros::ok())
    {
        
        if(sampled_points_.size() >= num_points) {break;}

        float r_ = normal_dis(gen);
        float theta_ = theta_dis(gen);

        //Point p1 = {c_.x + r_ * cos(theta_), c_.y + r_ * sin(theta_)};
        Point p1 = Point{x_dis(gen), y_dis(gen)};
        
        Point p2 = {p1.x + r_ * cos(theta_), p1.y + r_ * sin(theta_)};

        r_ = normal_dis(gen); 
        theta_ = theta_dis(gen); 

        Point p3 = {p1.x + r_ * cos(theta_), p1.y + r_ * sin(theta_)};

        bool p1_free_ = robot_->isConfigurationFree(p1.x, p1.y);
        bool p2_free_ = robot_->isConfigurationFree(p2.x, p2.y);
        bool p3_free_ = robot_->isConfigurationFree(p3.x, p3.y);
        
        Point pt_; // point to insert
        bool valid_ = false; 

        // bool flags to check if points inside white polygon
        bool w1_ = robot_->isConfigurationFree(p1.x, p1.y); 
        bool w2_ = robot_->isConfigurationFree(p2.x, p2.y); 
        bool w3_ = robot_->isConfigurationFree(p3.x, p3.y); 
         
        // bool flags to check if points inside green polygon
        bool g1_ = poly_.isInsideGreenPolygon(Point_t(p1.x, p1.y)); 
        bool g2_ = poly_.isInsideGreenPolygon(Point_t(p2.x, p2.y)); 
        bool g3_ = poly_.isInsideGreenPolygon(Point_t(p3.x, p3.y)); 
        
        if(w1_)
        {
            if(!(w2_ || w3_) && (g2_ ^ g3_))
            {
                valid_ = true; 
                pt_ = p1; 
                
            }
        } 
        else if(w2_)
        {
            if(!(w3_ || w1_) && (g1_ ^ g3_))
            {
                valid_ = true; 
                pt_ = p2; 
                
            }
        } 
        else if(w3_)
        {
            if(!(w1_ || w2_) && (g2_ ^ g1_))
            {
                valid_ = true; 
                pt_ = p3; 
                
            }
        } 

        if(valid_)
        {
            sampled_points_.insert(Node2d{pt_.x, pt_.y});
        
        }
    }

    auto end_time = std::chrono::system_clock::now();
    auto elapsed  = std::chrono::duration_cast<std::chrono::seconds>(end_time - start_time);
    
    ROS_WARN("================================================================================") ;
    ROS_WARN("Sampling of %d points for row transition took %ld seconds!", cnt, elapsed.count());
    ROS_WARN("================================================================================") ;
    
    std::vector<Node2d> points_;
    points_.reserve(sampled_points_.size());
    for(const auto t: sampled_points_) {points_.push_back(t);}

    return points_;
    
}





std::vector<PRM::Node2d> PRM::Sampler::uniformSampleInRange(const std::vector<float> &rx, const std::vector<float> &ry, \
                                                        const int num_points)
{
    std::random_device rd;
    std::mt19937 gen(rd());
    
    int cnt =0 ; 

    std::vector<Node2d> points_;
    
    float x_mn = *std::min_element(rx.begin(), rx.end());
    float x_mx = *std::min_element(rx.begin(), rx.end());

    float y_mn = *std::min_element(ry.begin(), ry.end());
    float y_mx = *std::min_element(ry.begin(), ry.end());

    std::uniform_real_distribution<float > x_dis(x_mn - 5, x_mx + 5); 
    std::uniform_real_distribution<float > y_dis(y_mn - 50, y_mx + 50); 

    while(ros::ok() && cnt < num_points)
    {

        //float r_ = r_dis(gen);
        //float theta_ = theta(gen);

        float x = x_dis(gen);
        float y = y_dis(gen);

        Point p = {x, y};

        bool flag = robot_->isConfigurationFree(p.x, p.y);

        if(!flag) {continue;}

        const Node2d node_{p.x, p.y};
        
        points_.push_back(node_);

        cnt++;

    }
    
    return points_;

}

std::vector<PRM::Node2d> PRM::Sampler::uniformSamplingForRunway(const geometry_msgs::PoseStamped &start_pose_, \
                                                        const geometry_msgs::PoseStamped &goal_pose_, 
                                                        const int num_points)
{

    //ROS_WARN("=== Uniform Sample! ====");
    std::vector<Node2d> start_points_ , goal_points_, points_;

    geometry_msgs::PoseStamped lowest_pose_; //lowest pose for start and goal poses 

    bool f1_; 
    Point pt_, lowest_pt_;
    std::vector<float> vx, vy;

    // === processing start pose
    f1_ = getLowestPoint(start_pose_, goal_pose_, start_pose_, lowest_pose_);
    
    pt_ = Point{start_pose_.pose.position.x, start_pose_.pose.position.y}; 
    lowest_pt_ = Point{lowest_pose_.pose.position.x, lowest_pose_.pose.position.y}; 
    
    if(!f1_)
    {

        ROS_ERROR("Unable to find lowest point for start pose!");
        return std::vector<Node2d>();
    }
    
    
    vx = std::vector<float>{pt_.x,  lowest_pt_.x}; 
    vy = std::vector<float> {pt_.y,  lowest_pt_.y};

    //ROS_INFO("Before uniformSampleInRange()");
    start_points_ = uniformSampleInRange(vx, vy, num_points);
    //ROS_INFO("After uniformSampleInRange()");

    // === processing goal pose
    f1_ = getLowestPoint(start_pose_, goal_pose_, goal_pose_, lowest_pose_);
    
    pt_ = Point{goal_pose_.pose.position.x, goal_pose_.pose.position.y}; 
    lowest_pt_ = Point{lowest_pose_.pose.position.x, lowest_pose_.pose.position.y}; 
    
    if(!f1_)
    {

        ROS_ERROR("Unable to find lowest point for goal pose!");
        return std::vector<Node2d>();
    }
    
    
    vx = std::vector<float> {pt_.x,  lowest_pt_.x}; 
    vy = std::vector<float> {pt_.y,  lowest_pt_.y};

    goal_points_ = uniformSampleInRange(vx, vy, num_points);

   // ROS_WARN("=== Exiting Uniform Sample! ====");

    //return std::vector<float>{start_points_, goal_points_};

    for(auto &t : start_points_) points_.push_back(t); 
    for(auto &t : goal_points_) points_.push_back(t);

    return points_;
    
}




std::vector<PRM::Node2d> PRM::Sampler::gaussianSampleAlongWhitePolygon(const geometry_msgs::PoseStamped &start_pose_, \
                                                        const geometry_msgs::PoseStamped &goal_pose_, 
                                                        const int num_points)
{

    //== 'section' => horizontal section between the start and the goal row 

    //ROS_WARN("=== Gaussian Sample! ====");

    geometry_msgs::PoseStamped section_centre_; 

    //std::cout << "start_pose: (" << start_pose_.pose.position.x << "," << start_pose_.pose.position.y << ")" << std::endl;
    //std::cout << "goal_pose: (" << goal_pose_.pose.position.x << "," << goal_pose_.pose.position.y << ")" << std::endl;
    
   // bool flag_ = getPolygonCenter(start_pose_, goal_pose_, section_centre_);

    geometry_msgs::PoseStamped l1_, l2_; //lowest pose for start and goal poses 
    bool f1_, f2_; 
    
    f1_ = getLowestPoint(start_pose_, goal_pose_, start_pose_, l1_);
    f2_ = getLowestPoint(start_pose_, goal_pose_, goal_pose_, l2_);

    if(!f1_ || !f2_)
    {
        ROS_ERROR("unable to find lowest point for start OR goal pose!");
        return std::vector<Node2d>();
    }

    Point st_pt{start_pose_.pose.position.x, start_pose_.pose.position.y}; 
    Point go_pt{goal_pose_.pose.position.x, goal_pose_.pose.position.y};

    Point l1_pt{l1_.pose.position.x, l1_.pose.position.y}; 
    Point l2_pt{l2_.pose.position.x, l2_.pose.position.y};

    float x_mx, x_mn; 
    float y_mx, y_mn; 

    std::vector<float> vx{st_pt.x, go_pt.x , l1_pt.x, l2_pt.x}; 
    std::vector<float> vy{st_pt.y, go_pt.y, l1_pt.y, l2_pt.y};

    x_mx = *std::max_element(vx.begin()  , vx.end());
    x_mn = *std::min_element(vx.begin(), vx.end()); 

    y_mx = *std::max_element(vy.begin()  , vy.end());
    y_mn = *std::min_element(vy.begin(), vy.end()); 

    std::random_device rd;
    std::mt19937 gen(rd());
    
    //std::uniform_real_distribution<float > x_dis(p_centre.x - 0.5 * section_width * cos(theta), p_centre.x +  0.5 * section_width * cos(theta)); 
    //std::uniform_real_distribution<float > y_dis(p_centre.y - section_len * sin(theta), p_centre.y +  section_len * sin(theta)); 

    std::uniform_real_distribution<float > x_dis(x_mn - 5, x_mx + 5); 
    std::uniform_real_distribution<float > y_dis(y_mn - 5, y_mx + 5); 
    std::uniform_real_distribution<float > theta_dis(0, 2 * M_PI); //theta distribution
    

    auto start_time = std::chrono::system_clock::now();
    // Perform some time-consuming operation
    int cnt= 0 ; 

    float sigma = 3.f; 
    std::normal_distribution<> normal_dis(0.f, sigma);


    sampled_points_.clear();

    CollisionDetectionPolygon &poly_ = robot_->getCollisionPolyRef();

    while(ros::ok())
    {
        
        if(sampled_points_.size() >= num_points) {break;}

        float r_ = normal_dis(gen);
        float theta_ = theta_dis(gen);

        //Point p1 = {c_.x + r_ * cos(theta_), c_.y + r_ * sin(theta_)};
        Point p1 = Point{x_dis(gen), y_dis(gen)};
        
        Point p2 = {p1.x + r_ * cos(theta_), p1.y + r_ * sin(theta_)};

        bool p1_free_ = robot_->isConfigurationFree(p1.x, p1.y);
        bool p2_free_ = robot_->isConfigurationFree(p2.x, p2.y);
    
        Point pt_; // point to insert
        bool valid_ = false; 

        if(p1_free_  && !p2_free_ && !poly_.isInsideGreenPolygon(Point_t(p2.x, p2.y)))  {
            pt_ = p1 ; 
            valid_ = true; 
        }

        else if (!p1_free_ && !poly_.isInsideGreenPolygon(Point_t(p1.x, p2.x))  && p2_free_) {
            pt_ = p2; 
            valid_ = true; 
        }

        if(valid_)
        {
            sampled_points_.insert(Node2d{pt_.x, pt_.y});
        
        }
    }

    auto end_time = std::chrono::system_clock::now();
    auto elapsed  = std::chrono::duration_cast<std::chrono::seconds>(end_time - start_time);
    
    ROS_WARN("================================================================================") ;
    ROS_WARN("Sampling of %d points for row transition took %ld seconds!", cnt, elapsed.count());
    ROS_WARN("================================================================================") ;
    
    std::vector<Node2d> points_;
    points_.reserve(sampled_points_.size());
    for(const auto t: sampled_points_) {points_.push_back(t);}



    return points_;
    
}



std::vector<PRM::Node2d> PRM::Sampler::uniformSample(const geometry_msgs::PoseStamped &start_pose_, \
                                                        const geometry_msgs::PoseStamped &goal_pose_, 
                                                        const int num_points)
{

    //== 'section' => horizontal section between the start and the goal row 

    //ROS_WARN("=== Gaussian Sample! ====");

    geometry_msgs::PoseStamped section_centre_; 

    //std::cout << "start_pose: (" << start_pose_.pose.position.x << "," << start_pose_.pose.position.y << ")" << std::endl;
    //std::cout << "goal_pose: (" << goal_pose_.pose.position.x << "," << goal_pose_.pose.position.y << ")" << std::endl;
    
   // bool flag_ = getPolygonCenter(start_pose_, goal_pose_, section_centre_);

    geometry_msgs::PoseStamped l1_, l2_; //lowest pose for start and goal poses 
    bool f1_, f2_; 
    
    f1_ = getLowestPoint(start_pose_, goal_pose_, start_pose_, l1_);
    f2_ = getLowestPoint(start_pose_, goal_pose_, goal_pose_, l2_);

    if(!f1_ || !f2_)
    {
        ROS_ERROR("unable to find lowest point for start OR goal pose!");
        return std::vector<Node2d>();
    }

    Point st_pt{start_pose_.pose.position.x, start_pose_.pose.position.y}; 
    Point go_pt{goal_pose_.pose.position.x, goal_pose_.pose.position.y};

    Point l1_pt{l1_.pose.position.x, l1_.pose.position.y}; 
    Point l2_pt{l2_.pose.position.x, l2_.pose.position.y};

    float x_mx, x_mn; 
    float y_mx, y_mn; 

    std::vector<float> vx{st_pt.x, go_pt.x , l1_pt.x, l2_pt.x}; 
    std::vector<float> vy{st_pt.y, go_pt.y, l1_pt.y, l2_pt.y};

    x_mx = *std::max_element(vx.begin()  , vx.end());
    x_mn = *std::min_element(vx.begin(), vx.end()); 

    y_mx = *std::max_element(vy.begin()  , vy.end());
    y_mn = *std::min_element(vy.begin(), vy.end()); 

    std::random_device rd;
    std::mt19937 gen(rd());
    
    //std::uniform_real_distribution<float > x_dis(p_centre.x - 0.5 * section_width * cos(theta), p_centre.x +  0.5 * section_width * cos(theta)); 
    //std::uniform_real_distribution<float > y_dis(p_centre.y - section_len * sin(theta), p_centre.y +  section_len * sin(theta)); 

    std::uniform_real_distribution<float > x_dis(x_mn - 5, x_mx + 5); 
    std::uniform_real_distribution<float > y_dis(y_mn - 5, y_mx + 5); 


    auto start_time = std::chrono::system_clock::now();
    // Perform some time-consuming operation
    int cnt= 0 ; 

    sampled_points_.clear();
    
    while(ros::ok() && cnt < num_points)
    {

        //float r_ = r_dis(gen);
        //float theta_ = theta(gen);

        float x = x_dis(gen);
        float y = y_dis(gen);

        Point p = {x, y};

        bool flag = robot_->isConfigurationFree(p.x, p.y);

        if(!flag) {continue;}

        const Node2d node_{p.x, p.y};
        if(sampled_points_.count(node_) == 0)
        {
            sampled_points_.insert(node_);
            cnt++;
        }

    }

    auto end_time = std::chrono::system_clock::now();
    auto elapsed  = std::chrono::duration_cast<std::chrono::seconds>(end_time - start_time);
    
    ROS_WARN("================================================================================") ;
    ROS_WARN("Sampling of %d points for row transition took %ld seconds!", cnt, elapsed.count());
    ROS_WARN("================================================================================") ;
    
    std::vector<Node2d> points_;
    points_.reserve(sampled_points_.size());
    for(const auto t: sampled_points_) {points_.push_back(t);}



    return points_;
    
}

bool PRM::Sampler::getLowestPoint(const geometry_msgs::PoseStamped &start_, const geometry_msgs::PoseStamped &goal, \
                                geometry_msgs::PoseStamped target_pose_, geometry_msgs::PoseStamped &lowest_pose) 
{

    //ROS_WARN("======== Inside PRM::Sampler::getPolygonCenter() ========");   
    Point p1{start_.pose.position.x, start_.pose.position.y};
    Point p2{goal.pose.position.x, goal.pose.position.y};

    //Point mid((p1.x + p2.x) / 2, (p1.y + p2.y) / 2);

    float theta_ = std::atan2((p1.x - p2.x),  (p2.y - p1.y));

    //ROS_INFO("theta_: %f", theta_ * 180 / M_PI);

    float sz_ = 0.1;  //step-size 

    int limit = 500; 
    int iter_cnt = 0 ; 
    bool found = false;

    Point end_;
    //float dir_; //direction 

    // === direction detection
    //dir_ = getCorrectDirection(mid, theta_, sz_);

    Point target = {target_pose_.pose.position.x, target_pose_.pose.position.y};

    CollisionDetectionPolygon &p = robot_->getCollisionPolyRef(); 
    
    while(ros::ok() && iter_cnt < limit)
    {   
        iter_cnt++; 

        Point pt;

        float x1 = target.x +  iter_cnt * sz_ *  std::cos(theta_);
        float y1 = target.y +  iter_cnt * sz_ * std::sin(theta_);

        if(!p.isInsideGreenPolygon(Point_t{x1, y1}) && !p.isConfigurationFree(x1, y1)) {

            end_.x = x1;
            end_.y = y1;
            found = true;
            break; 
        }
        
        float x2 = target.x -  iter_cnt * sz_ *  std::cos(theta_);
        float y2 = target.y -  iter_cnt * sz_ * std::sin(theta_);

        if(!p.isInsideGreenPolygon(Point_t{x2, y2}) && !p.isConfigurationFree(x2, y2)) {

            end_.x = x2;
            end_.y = y2;
            found = true;
            break; 
        }
        
        
    }
    
    if(found)
    {
    // Point centre_ = Point((mid.x + end_.x) / 2.f, (mid.y + end_.y) / 2.f);
        
        lowest_pose.header.frame_id = "map";
        lowest_pose.header.stamp = ros::Time::now();
        lowest_pose.pose.position.x = end_.x;
        lowest_pose.pose.position.y = end_.y;
        lowest_pose.pose.orientation = Utils::getQuatFromYaw(theta_);

        lowest_pose_pub.publish(lowest_pose);
    }
    
    //ROS_WARN(" End of PRM::Sampler::getPolygonCenter() function!");
    return found;
}


std::vector<PRM::Node2d> PRM::Sampler::gaussianSamplePointsForRowTransition(const geometry_msgs::PoseStamped &start, 
                                                const geometry_msgs::PoseStamped &goal,
                                                const int num_points)
{

    sampled_points_.clear();


    struct Point{
        float x;
        float y;
    };
    //robot_->getOBB({0,0},0);
    Point s_ = {start.pose.position.x, start.pose.position.y};
    Point g_ = {goal.pose.position.x, goal.pose.position.y};

    Point c_ = {(s_.x + g_.x) / 2, (s_.y + g_.y) / 2};
    
    float d_ = sqrt(pow(s_.x - g_.x, 2) + pow(s_.y - g_.y, 2));
    
    int cnt = 0 ; 

    std::random_device rd;
    std::mt19937 gen(rd());
    
    std::uniform_real_distribution<float > r_dis(0, 2.4 * 0.5 * d_); //radius distribution
    std::uniform_real_distribution<float > theta(0, 2 * M_PI); //theta distribution
    
    std::normal_distribution<> normal_dis(0.f, 3.f);


    //std::vector<Point> points_;
    auto start_time = std::chrono::system_clock::now();
    // Perform some time-consuming operation
    
    int counter =0 ; 
    while(ros::ok())
    {
        
        if(sampled_points_.size() >= num_points) {break;}

        float r_ = r_dis(gen);
        float theta_ = theta(gen);

        Point p1 = {c_.x + r_ * cos(theta_), c_.y + r_ * sin(theta_)};

        r_ = normal_dis(gen); //radius for the second point
        theta_ = theta(gen); //theta for the second point

        Point p2 = {p1.x + r_ * cos(theta_), p1.y + r_ * sin(theta_)};

        bool p1_free_ = robot_->isConfigurationFree(p1.x, p1.y);
        bool p2_free_ = robot_->isConfigurationFree(p2.x, p2.y);
    
        Point pt_; // point to insert
        bool valid_ = false; 
        if(p1_free_ && !p2_free_) {
            pt_ = p1 ; 
            valid_ = true; 
        }

        else if (!p1_free_ && p2_free_) {
            pt_ = p2; 
            valid_ = true; 
        }

        if(valid_)
        {
            sampled_points_.insert(Node2d{pt_.x, pt_.y});
        
        }
        

    }

    auto end_time = std::chrono::system_clock::now();
    auto elapsed  = std::chrono::duration_cast<std::chrono::seconds>(end_time - start_time);
    
    ROS_WARN("================================================================================") ;
    ROS_WARN("Sampling of %d points for row transition took %ld seconds!", cnt, elapsed.count());
    ROS_WARN("================================================================================") ;
    
    std::vector<Node2d> points_;
    points_.reserve(sampled_points_.size());
    for(const auto t: sampled_points_) {points_.push_back(t);}

    return points_;

}



std::vector<PRM::Node2d> PRM::Sampler::samplePointsForRowTransition(const geometry_msgs::PoseStamped &start, 
                                                const geometry_msgs::PoseStamped &goal,
                                                const int num_points)
{

    sampled_points_.clear();


    struct Point{
        float x;
        float y;
    };

    //robot_->getOBB({0,0},0);
    Point s_ = {start.pose.position.x, start.pose.position.y};
    Point g_ = {goal.pose.position.x, goal.pose.position.y};

    Point c_ = {(s_.x + g_.x) / 2, (s_.y + g_.y) / 2};
    
    float d_ = sqrt(pow(s_.x - g_.x, 2) + pow(s_.y - g_.y, 2));
    
    int cnt = 0 ; 

    std::random_device rd;
    std::mt19937 gen(rd());
    
    std::uniform_real_distribution<float > r_dis(0, 2.4 * 0.5 * d_); //radius distribution
    std::uniform_real_distribution<float > theta(0, 2 * M_PI); //theta distribution
    
    //std::vector<Point> points_;
    auto start_time = std::chrono::system_clock::now();
    // Perform some time-consuming operation
    
    while(ros::ok() && cnt < num_points)
    {

        float r_ = r_dis(gen);
        float theta_ = theta(gen);

        Point p = {c_.x + r_ * cos(theta_), c_.y + r_ * sin(theta_)};

        bool flag = robot_->isConfigurationFree(p.x, p.y);

        if(!flag) {continue;}

        const Node2d node_{p.x, p.y};
        if(sampled_points_.count(node_) == 0)
        {
            sampled_points_.insert(node_);
            cnt++;
        }

    }

    auto end_time = std::chrono::system_clock::now();
    auto elapsed  = std::chrono::duration_cast<std::chrono::seconds>(end_time - start_time);
    
    ROS_WARN("================================================================================") ;
    ROS_WARN("Sampling of %d points for row transition took %ld seconds!", cnt, elapsed.count());
    ROS_WARN("================================================================================") ;
    
    std::vector<Node2d> points_;
    points_.reserve(sampled_points_.size());
    for(const auto t: sampled_points_) {points_.push_back(t);}



    return points_;

}

/*void PRM::Sampler::sampledPointsCallback(geometry_msgs::PoseArrayPtr msg)
{

    ROS_INFO("Inside PRM::Sampler::sampledPointsCallback!");

    sampled_points_.clear();

    std::vector<Node2d> points_;
    points_.reserve(Constants::Planner::N_ + 1000);

    for(const auto &t: msg->poses)
    {   
        const float  x_ = t.position.x;
        const float  y_ = t.position.y;

        const Node2d node_{x_, y_};

        if(sampled_points_.find(node_) == sampled_points_.end())
        {

            sampled_points_.insert(node_);
            //points_.push_back(node_);
    
        }
   

    }

    is_ready  = true; 
     

}*/

/*std::vector<PRM::Node2d> PRM::Sampler::generate2DSamplePoints()
{      

    while(ros::ok() && !is_ready)
    {
        ROS_INFO("Waiting for sampled points!");
        ros::spinOnce();
        ros::Duration(0.5).sleep();
        
    }


    ROS_DEBUG("Inside generate2Dsamplepoints function!");
    std::vector<Node2d> points;
    for (const auto &t: sampled_points_)
    {   
        points.push_back(t);
        //ROS_INFO("(x,y) => (%f,%f)", t.x_, t.y_);
    }

     
    geometry_msgs::PoseArray pose_array_;
    pose_array_.header.frame_id = "map"; 
    pose_array_.header.stamp = ros::Time::now(); 



    for(const auto &t: sampled_points_) 
    {
        
        geometry_msgs::Pose pose_; 
        
        pose_.position.x = t.x_;
        pose_.position.y = t.y_; 
        pose_.orientation = Utils::getQuatFromYaw(0.f);

        pose_array_.poses.push_back(pose_);

    }   

    visualize_->publishT<geometry_msgs::PoseArray>("sampled_points", pose_array_);
    
    return points;
}*/

/*std::vector<PRM::Node2d> PRM::Sampler::generate2DSamplePoints()
{

    //ROS_INFO("Inside simplePRM::samplePoints!");

    ROS_INFO("Sampling of %d points started!", Constants::Planner::N_);
    auto start = std::chrono::high_resolution_clock::now();

    
    const int w_ = Constants::MapMetaData::width_; 
    const int h_ = Constants::MapMetaData::height_;

    ROS_INFO("map_dimension: (%d,%d)", h_, w_);

    //range of x in real world and NOT GRID
    std::vector<float> rx_ = {Constants::MapMetaData::origin_x_ , Constants::MapMetaData::origin_x_ + (w_ + 0.5f) * Constants::MapMetaData::res_}; 
    std::vector<float> ry_ = {Constants::MapMetaData::origin_y_ , Constants::MapMetaData::origin_y_ + (h_ + 0.5f) * Constants::MapMetaData::res_} ;
    
    
    //std::random_device rd;
    unsigned int seed_ = 2; 
    std::mt19937 gen(seed_);
    
    
    std::uniform_real_distribution<float> dist_x(rx_[0] + 100, rx_[0] + 150);
    std::uniform_real_distribution<float> dist_y(ry_[0] + 100, ry_[0] + 150);
     
    int cnt_ =0 ; 

    std::vector<Node2d> points_;
    points_.reserve(Constants::Planner::N_ + 1000);

    while(sampled_points_.size() < Constants::Planner::N_ && ros::ok())  
    {
        //ROS_INFO("nodes2d_.size(): %d", nodes2d_.size());
        const float  x_ = dist_x(gen);
        const float  y_ = dist_y(gen);

        //ROS_INFO("(x,y) => (%f,%f)", x_, y_);

        cnt_++; 
        const Node2d node_{x_, y_};

        if(sampled_points_.find(node_) == sampled_points_.end())
        {

            sampled_points_.insert(node_);
            points_.push_back(node_);

        }
    }

    
    geometry_msgs::PoseArray pose_array_;
    pose_array_.header.frame_id = "map"; 
    pose_array_.header.stamp = ros::Time::now(); 



    for(const auto &t: sampled_points_) 
    {
        
        geometry_msgs::Pose pose_; 
        
        pose_.position.x = t.x_;
        pose_.position.y = t.y_; 
        pose_.orientation = Utils::getQuatFromYaw(0.f);

        pose_array_.poses.push_back(pose_);

    }   

    visualize_->publishT<geometry_msgs::PoseArray>("sampled_points", pose_array_);
    
    return points_; 

}*/
