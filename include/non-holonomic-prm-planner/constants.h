#ifndef CONSTANTS_H
#define CONSTANTS_H

#include <string>
#include <cmath>

/**
 * Unit Test Cases
 * Github documentation
 * Detailed commnets   + DOYGEN
 * Narrow corridor support
 * Smoothing
 * Enhanced sampling
 * parameterize + disable reverse
 * Tune params
*/

namespace PRM{



    namespace Constants{
        
        static const bool manual = true;
        
        static const std::string &map_frame_id = "map";




        struct MapMetaData {
            
            
            static  float cell_size_, origin_x_, origin_y_, res_;
            static  int  height_, width_;
            
        };
        

        static const std::string &map_topic = "/autodrive/occupancy_grid";

        namespace Vehicle {
        
            static const float delta_max_ = 60 *M_PI / 180.0;  //max steering angle in radians
            static const float a2_ = 0.8; //len_to_hitch 
            static const float l_ = 1.84; //wheelbase
            static const float R_MIN_ = sqrt(pow(a2_, 2) + pow(l_ * (1.f/std::tan(delta_max_)), 2)); //minimum turning radius
           
            static const float wheelbase_ =  1.84f;
            static const float front_length_ = 2.92;
            static const float hitch_length_ = 0.8;
            static const float left_width_ = 0.8;
            static const float right_width_ = 0.8; 
            
        };

        namespace Planner{

            
            static const int N_ = 2000;
            static const float theta_sep_ = 100.f * M_PI / 180.f;  //5 degree separation
            static const float dis_sep_ = 0.1f; //10  cm

            static const bool can_reverse_ = false;
            
            static const float max_res_ = sqrt(pow(Vehicle::R_MIN_,2) + pow(Vehicle::R_MIN_ - Vehicle::a2_,2)); // max allowed resolution
            //static const float theta_tol_ = 2.5 * M_PI / 180.0 ;  // 2.5 degrees 
            //static const float theta_tol_ = 0.5 * M_PI / 180.0 ;  // 2.5 degrees 
            static const float theta_tol_ =  0.5f * theta_sep_ ;  // 2.5 degrees 
            static const float planner_res_ = 0.5;

            static const float w_dis_  = 90; 
            static const float w_ang_ = 0.1; 
            static const float w_rev_ = 1000.f  ;   //reverse path cost weightage

            static const float sr_ = max_res_;

        };



        
        //** vehicle params
            
        








    };





};




#endif