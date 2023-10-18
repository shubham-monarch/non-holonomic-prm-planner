#include <non-holonomic-prm-planner/robot_model.h>




PRM::RobotModel::RobotModel(    const float front_len, const float hitch_len, \
                                const float left_width, const float right_width):  len_to_front(front_len),\
                                                                                len_to_hitch(hitch_len), \
                                                                                width_left(left_width), \
                                                                                width_right(right_width)
{   
    ROS_INFO("Inside RobotModel constructor!");
    //cdp_ = std::make_shared<CollisionDetectionPolygon>();

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
    
    /*std::array<float,8> dim = {
        len_to_front,
        width_left,
        width_right,
        len_to_hitch
    };*/
    
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

    /*ROS_INFO("Inside getOBB() ===> : (%f,%f,%f)", pos[0], pos[1], heading);

    float f_lx = pos[0] + dim[0] * std::cos(heading) + dim[1] * std::cos(M_PI/2.f + heading);
    float f_ly = pos[1] + dim[0] * std::sin(heading) + dim[1] * std::sin(M_PI/2.f + heading); 
    
    float f_rx = pos[0] + dim[0] * std::cos(heading) + dim[2] * std::cos(heading - M_PI/2.f);
    float f_ry = pos[1] + dim[0] * std::sin(heading) + dim[2] * std::sin(heading - M_PI/2.f);

    float b_lx = pos[0] - dim[3] * std::cos(heading) + dim[1] * std::cos(M_PI +  heading - M_PI/2.f);
    float b_ly = pos[1] - dim[3] * std::sin(heading) + dim[1] * std::sin(M_PI + heading - M_PI/2.f);
    
    float b_rx = pos[0] - dim[3] * std::cos(heading) + dim[2] * std::cos(M_PI + heading + M_PI/2.f);
    float b_ry = pos[1] - dim[3] * std::sin(heading) + dim[2] * std::sin(M_PI + heading + M_PI/2.f);

    const std::vector<float> &obb = {f_lx, f_ly, f_rx, f_ry, b_rx, b_ry, b_lx, b_ly};
    */
    return obb;
}




bool PRM::RobotModel::isConfigurationFree(const std::vector<float> &obb, bool publish) const {
    
    if (!collision_p.isConfigurationFree(obb, publish)) {
        
        return false;
    }

    
    
    return true;
}

bool PRM::RobotModel::isConfigurationFree(const float x, const float y) const {
    
    //std::array<float,2> position = {x,y};
    //std::vector<float> obb = getOBB(position,0);
    return collision_p.isConfigurationFree(x, y);
}