#ifndef PRM_PLANNER_H
#define PRM_PLANNER_H

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>

#include <non-holonomic-prm-planner/visualizations.h>

/**
 * Simple PRM implementation
*/


//SubRoutines ==> 
//Sampling
//Resampling
//Local planner
//CollisionDetection ***
//Distance Metrics
//k-Nearest neighbours


namespace PRM{
    
    class Visualize;

    //represents a sampled point with no heading information
    struct Node2d {

        int x_, y_; 
        float theta_;

        Node2d(const int x, const int y) : x_(x), y_(y){}
    
        
    };

    //kdtree adapter
    struct Node2dAdapter {
        
        typedef float ElementType;

        const Node2d& obj;

        explicit Node2dAdapter(const Node2d& obj) : obj(obj) {}

        inline ElementType operator[](size_t index) const {
            switch (index) {
                case 0: return obj.x_;
                case 1: return obj.y_;
                //case 2: return obj.z;
                default: throw std::out_of_range("Invalid index for MyClassAdapter");
            }
        }
    };
    
        
    class SimplePRM{

        public:

       
            SimplePRM();

            
            void setMapCb(nav_msgs::OccupancyGrid::ConstPtr map_);
            void initialize();
            bool generateRoadMap();


        private: 


            // ===== functions ====

            bool buildKDtree();

            bool generateSamplePoints();
            bool isObstacleFree(const Node2d &node_) const;     
            
            bool generateSteeringCurve(geometry_msgs::Pose robot_pose_,  float delta_);

            bool generateSteeringCurve(geometry_msgs::Pose robot_pose_,  geometry_msgs::Pose &config_pose_, float R_);
            
            
            void generateSteeringCurveFamily(geometry_msgs::Pose robot_pose_);
            
            bool connectConfigurationToRobot(geometry_msgs::Pose rp_, geometry_msgs::Pose configuration_)   ;

           
            //==== variables =====
            Visualize visualize_;
            bool map_set_;
            nav_msgs::OccupancyGridConstPtr grid_;



            //** planner tuning params
            int N_ = 100;  //number of initial poses
            
            
            
            //core planner vars
            std::vector<Node2d> nodes2d_;
            std::vector<geometry_msgs::Pose> steering_curve_family_poses_;


            //** ROS members
            ros::NodeHandle nh_;
            ros::Subscriber map_sub_;
            


    };


};

#endif