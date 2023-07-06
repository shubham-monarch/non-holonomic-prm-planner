#include <non-holonomic-prm-planner/planner.h>
#include <non-holonomic-prm-planner/collisiondetectionpolygon.h>

#include <ros/console.h>

int main(int argc, char** argv) {


    ros::init(argc, argv, "PRM");
    
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);

    //PRM::Planner prm_;

    PRM::CollisionDetectionPolygon cdp_;
    cdp_.initialize();
    


    ros::spin();

    return 0;

}
