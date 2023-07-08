#ifndef HELPER_H
#define HELPER_H

#include <non-holonomic-prm-planner/constants.h>

namespace PRM{

    namespace Helper{

        static inline void mapToWorld(const int mx, const int my, float& wx, float& wy) 
        {   

            float origin_x_ = Constants::MapMetaData::origin_x_; 
            float origin_y_ = Constants::MapMetaData::origin_y_; 
            
            float resolution_ = Constants::MapMetaData::cell_size_;

            wx = origin_x_ + (mx + 0.5) * resolution_;
            wy = origin_y_ + (my + 0.5) * resolution_;
        
        };


        //delta ==> steering angle
        //TODO: Check delta != (2n + 1)*pi/2
        static inline float calculateR(const float a2, const float wb, const float delta_){

            // R(delta_) = sqrt(a2 * a2 + l* l * cot(delta) * cot(delta))

            const float t1_ = a2 * a2; 

            const float t2_ = pow(wb, 2)  * (1.f / pow(std::tan(delta_), 2));


            const float R_ = sqrt(t1_ + t2_);

            return R_; 
        } 

        



    };


};


#endif