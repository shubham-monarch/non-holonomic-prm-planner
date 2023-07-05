#include <non-holonomic-prm-planner/planner.h>


int main(int argc, char** argv) {


    ros::init(argc, argv, "PRM");
    PRM::Planner prm_;

    ros::spin();
    return 0;

}
