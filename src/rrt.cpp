#include <non-holonomic-prm-planner/rrt.h>

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
        polygon_set_ = true; 
    }
    else
    {
        ROS_ERROR("polygon is not valid!");
        return;
    }

}

void PRM::rrt::reset()
{
    goal_pose_set_ = false; 
    start_pose_set_ = false;
    polygon_set_ = false;
}





bool PRM::rrt::plan(const geometry_msgs::PoseStamped &start_pose_, const geometry_msgs::PoseStamped &goal_pose_)
{   
    if(!goal_pose_set_ || !start_pose_set_)
    {
        ROS_ERROR("start or goal pose is not set!");
        return false;
    }

    tree_.push_back(rrt_node{start_pose_});

    while(ros::ok())
    {




    }






}




