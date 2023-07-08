#ifndef CONSTANTS
#define CONSTANTS

#include <string>


//#include <non-holonomic-prm-planner/helper.h>



namespace PRM{



    namespace Constants{
        
        static const bool manual = true;
        
        static const std::string &map_frame_id = "map";




        struct MapMetaData {
            
            
            inline static float cell_size_, origin_x_, origin_y_;
            inline static unsigned int height_, width_;
            

        };
        

        static const std::string &map_topic = "/autodrive/occupancy_grid";

        
        //** vehicle params
            
        








    };





};




#endif