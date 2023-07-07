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
        
        }

        



    };


};


#endif