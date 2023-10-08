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

    centre_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("centre_pose", 1, true);
}



bool PRM::Sampler::getPolygonCenter(const geometry_msgs::PoseStamped &start_, const geometry_msgs::PoseStamped &goal, Point &centre_)
{

    //ROS_WARN("======== Inside PRM::Sampler::getPolygonCenter() ========");   
    Point p1{start_.pose.position.x, start_.pose.position.y};
    Point p2{goal.pose.position.x, goal.pose.position.y};

    Point mid((p1.x + p2.x) / 2, (p1.y + p2.y) / 2);

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

    CollisionDetectionPolygon &p = robot_->getCollisionPolyRef(); 
    while(ros::ok() && iter_cnt < limit)
    {   
        iter_cnt++; 

        Point pt;

        float x1 = mid.x +  iter_cnt * sz_ *  std::cos(theta_);
        float y1 = mid.y +  iter_cnt * sz_ * std::sin(theta_);

        if(!p.isInsideGreenPolygon(Point_t{x1, y1}) && !p.isConfigurationFree(x1, y1)) {

            end_.x = x1;
            end_.y = y1;
            found = true;
            break; 
        }
        
        float x2 = mid.x -  iter_cnt * sz_ *  std::cos(theta_);
        float y2 = mid.y -  iter_cnt * sz_ * std::sin(theta_);

        if(!p.isInsideGreenPolygon(Point_t{x2, y2}) && !p.isConfigurationFree(x2, y2)) {

            end_.x = x2;
            end_.y = y2;
            found = true;
            break; 
        }
        
        
    }

    if(found)
    {
        
        centre_ = Point((mid.x + end_.x) / 2.f, (mid.y + end_.y) / 2.f);
        geometry_msgs::PoseStamped pose_; 
        pose_.header.frame_id = "map";
        pose_.header.stamp = ros::Time::now();
        pose_.pose.position.x = centre_.x;
        pose_.pose.position.y = centre_.y;
        pose_.pose.orientation = Utils::getQuatFromYaw(theta_);

        centre_pose_pub_.publish(pose_);
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
