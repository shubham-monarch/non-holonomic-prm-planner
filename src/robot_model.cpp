#include <non-holonomic-prm-planner/robot_model.h>




PRM::RobotModel::RobotModel(    const float front_len, const float hitch_len, \
                                const float left_width, const float right_width):  len_to_front(front_len),\
                                                                                len_to_hitch(hitch_len), \
                                                                                width_left(left_width), \
                                                                                width_right(right_width)
{   
    ROS_INFO("Inside RobotModel constructor!");
    cdp_ = std::make_shared<CollisionDetectionPolygon>();
}

std::vector<float> PRM::RobotModel::getOBB(const std::array<float,2>& position, float heading) const {
    
    std::array<float,8> dim = {
        len_to_front,
        width_left,
        len_to_front,
        -width_right,
        -len_to_hitch,
        -width_right,
        -len_to_hitch,
        width_left
    };
    std::vector<float> obb;
    for (size_t i=0;i<dim.size();i+=2){
        obb.push_back(
            dim[i]*std::cos(heading)
            -dim[i+1]*std::sin(heading)
            +position[0]
        );
        obb.push_back(
            dim[i]*std::sin(heading)
            +dim[i+1]*std::cos(heading)
            +position[1]
        );
    }
    return obb;
}

bool PRM::RobotModel::isConfigurationFree(const std::vector<float> &obb_) const
{
    return true;
    return cdp_->isConfigurationFree(obb_);

} 



