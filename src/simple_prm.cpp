#include <non-holonomic-prm-planner/simple_prm.h>
#include <non-holonomic-prm-planner/constants.h>

#include <random>

PRM::SimplePRM::SimplePRM()
{   

    ROS_WARN("SimplePRM constructor called");

    
}

void PRM::SimplePRM::initialize()
{

    ros::Rate r_(5.0);
    map_sub_ = nh_.subscribe(Constants::map_topic, 1, &SimplePRM::setMapCb, this);
    
    while(ros::ok() && !map_set_){

        
        ros::spinOnce(); 
    
        r_.sleep();
    }

    ROS_WARN("map_set_ is true!");

    nodes_.clear();

    ros::Rate r(10);
    
    r_min_ = Helper::calculateR(a_2_, l_, delta_max_);
    max_res_ = sqrt(pow(r_min_,2) + pow(r_min_ - a_2_, 2));

    ROS_INFO("max_res_: %f", max_res_);
    ROS_INFO("min_r_: %f", r_min_);

    nodes_.clear();


}

bool PRM::SimplePRM::isObstacleFree(const Node &node_) const
{

    return true; 
}

bool PRM::SimplePRM::plan(){

    ROS_INFO("Inside PRM::plan()");

    samplePoints();

    return true; 
}

void PRM::SimplePRM::setMapCb(nav_msgs::OccupancyGrid::ConstPtr map_)
{

    Constants::MapMetaData::origin_x_ = map_->info.origin.position.x;
    Constants::MapMetaData::origin_y_ = map_->info.origin.position.y;
    Constants::MapMetaData::cell_size_ = map_->info.resolution;
    Constants::MapMetaData::height_ = map_->info.height;
    Constants::MapMetaData::width_ = map_->info.width;
    
    grid_ = map_;

    ROS_DEBUG("MAP PARAMS ===>");
    ROS_DEBUG("dimensions: (%d, %d)", map_->info.height, map_->info.width);
    ROS_DEBUG("resolution: %f", map_->info.resolution);
    ROS_DEBUG("origin: (%f, %f)", map_->info.origin.position.x, map_->info.origin.position.y);

    map_set_ = true; 
    
    
}


bool PRM::SimplePRM::samplePoints(){

    ROS_INFO("Inside simplePRM::samplePoints!");

    const int width_ = grid_->info.width; 
    const int height_ = grid_->info.height;

    ROS_INFO("grid_: (%f * %f)", height_, width_);

    std::random_device rd;
    std::mt19937 gen(rd());
    
    // Define the range for x, y, and theta
    std::uniform_int_distribution<int> dist_x(0, width_);
    std::uniform_int_distribution<int> dist_y(0, height_);
    std::uniform_real_distribution<float> dist_theta(0.0, 2 * M_PI);
    
    
    if((int)nodes_.size() > 0) {

        ROS_ERROR("nodes_.size() > 0!"); 
        return false;

    }
     
    nodes_.reserve(N_);

    while((int)nodes_.size()  < N_) {

        int x_ = dist_x(gen);
        int y_ = dist_y(gen);
        float theta_ = dist_theta(gen);
        
        Node node_(x_, y_, theta_);

        if(isObstacleFree(node_)) {

            nodes_.emplace_back(node_);

        }

    }
    
    ROS_DEBUG("nodes_.size(): %d", nodes_.size());
    
    
   visualize_.visualizeSampledPoints(nodes_);

    return true; 

}


