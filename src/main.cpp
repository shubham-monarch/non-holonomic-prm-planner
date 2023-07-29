#include <non-holonomic-prm-planner/roadmap.h>

#include <ros/console.h>


std::shared_ptr<PRM::Visualize> visualize_;
std::shared_ptr<PRM::RobotModel> robot_;

int main(int argc, char** argv) {


    ros::init(argc, argv, "PRM");
    
    visualize_ = std::make_shared<PRM::Visualize>();
    
    robot_  = std::make_shared<PRM::RobotModel>(PRM::Constants::Vehicle::front_length_, \
                                                PRM::Constants::Vehicle::hitch_length_, \
                                                PRM::Constants::Vehicle::left_width_, \
                                                PRM::Constants::Vehicle::right_width_);
        


    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);

    std::shared_ptr<PRM::Roadmap> roadmap_ = std::make_shared<PRM::Roadmap>();
    

    roadmap_->initialize();
    roadmap_->generateRoadMap();
    ros::spin();

    return 0;

}
