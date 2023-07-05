#ifndef CONSTANTS
#define CONSTANTS

#include <string>

namespace PRM{



    namespace Constants{
        
        static const bool manual = true;





        struct MapMetaData {
            static float cellSize, originX, originY;
            static unsigned int height, width;
        };
        

        static const std::string map_topic = "/autodrive/occupancy_grid";

        







    };





};




#endif