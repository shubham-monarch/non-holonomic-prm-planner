#ifndef ROBOT_MODEL_H
#define ROBOT_MODEL_H

#include <vector>
#include <array>

#include <non-holonomic-prm-planner/collisiondetectionpolygon.h>

namespace PRM
{

    class RobotModel
    {

       public: 

           RobotModel(  const float front_len, const float hitch_len, \
                        const float left_width, const float right_width);
            
           

            std::vector<float> getOBB(const std::array<float,2>& position, float heading) const;



            bool isConfigurationFree(const std::vector<float> &obb_) const; 

        private:

            float len_to_front, len_to_hitch;
            float width_left, width_right; 

            std::shared_ptr<CollisionDetectionPolygon> cdp_;

            //Visualize visualize_;

    };




    
};



#endif