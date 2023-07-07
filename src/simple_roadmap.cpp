#include <non-holonomic-prm-planner/simple_roadmap.h>
#include <random>

PRM::SimpleRoadmap::SimpleRoadmap():map_ready_(false)
{   
    ROS_DEBUG("Initializing SimpleRoadmap!");
    nodes_.clear();

    map_sub_ = nh_.subscribe(Constants::map_topic, 1, &SimpleRoadmap::setMapCb, this);

    ros::Rate r(10);
    
    while(!map_ready_) {
        
        //ROS_DEBUG("Map is not ready!");
        ros::spinOnce();
        r.sleep();
    
    }

    ROS_INFO("Map is ready!");
    
    
}

PRM::SimpleRoadmap::SimpleRoadmap(nav_msgs::OccupancyGridConstPtr &map_)
{

    grid_ = map_; 
    nodes_.clear();

}

bool PRM::SimpleRoadmap::isObstacleFree(const Node &node_) const
{

    return true; 
}

bool PRM::SimpleRoadmap::plan(){

    samplePoints();

    return true; 
}

void PRM::SimpleRoadmap::setMapCb(nav_msgs::OccupancyGrid::ConstPtr map_)
{

    Constants::MapMetaData::origin_x_ = map_->info.origin.position.x;
    Constants::MapMetaData::origin_y_ = map_->info.origin.position.y;
    Constants::MapMetaData::cell_size_ = map_->info.resolution;

    ROS_INFO("res: %f", Constants::MapMetaData::cell_size_);
    Constants::MapMetaData::height_ = map_->info.height;
    Constants::MapMetaData::width_ = map_->info.width;
    
    grid_ = map_;

    ROS_DEBUG("MAP PARAMS ===>");
    ROS_DEBUG("dimensions: (%d, %d)", map_->info.height, map_->info.width);
    ROS_DEBUG("resolution: %f", map_->info.resolution);
    ROS_DEBUG("origin: (%f, %f)", map_->info.origin.position.x, map_->info.origin.position.y);

    map_ready_ = true; 
    
    plan();
    
}

bool PRM::SimpleRoadmap::samplePoints(){

    const int width_ = grid_->info.width; 
    const int height_ = grid_->info.height;

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


