#include <non-holonomic-prm-planner/simple_prm.h>

#include <ros/console.h>


std::shared_ptr<PRM::Visualize> visualize_;

int main(int argc, char** argv) {


    ros::init(argc, argv, "PRM");
    
    visualize_ = std::make_shared<PRM::Visualize>();

    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);

    std::shared_ptr<PRM::SimplePRM> simple_prm_ = std::make_shared<PRM::SimplePRM>();
    

    simple_prm_->initialize();
    simple_prm_->generateRoadMap();
    ros::spin();

    return 0;

}
