#include <non-holonomic-prm-planner/planner.h>

#include <ros/console.h>

int main(int argc, char** argv) {


    ros::init(argc, argv, "PRM");
    
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);

    PRM::Planner prm_;

    ros::spin();

    return 0;

}
