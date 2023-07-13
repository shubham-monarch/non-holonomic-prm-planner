#ifndef PRM_PLANNER_H
#define PRM_PLANNER_H


#include <nav_msgs/OccupancyGrid.h>

#include <non-holonomic-prm-planner/visualizations.h>
#include <non-holonomic-prm-planner/KDTree.hpp>
//#include <non-holonomic-prm-planner/utils.h>
#include <non-holonomic-prm-planner/ds.h>


#include <unordered_set>



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
    
    
    typedef std::shared_ptr<kdTree::KDTree> kdTreePtr;
    typedef kdTree::point_t kdPoint;
    typedef kdTree::pointVec kdPoints;

    class Visualize;

    //represents a sampled point with no heading information in the GRID and not WORLD
        

    class SimplePRM{

        public:

       
            SimplePRM();

            
            void setMapCb(nav_msgs::OccupancyGrid::ConstPtr map_);
            void initialize();
            bool generateRoadMap();    


        private: 


            // ===== functions ====


            //**** core planner functions
            long long int set_N();
            float set_SR(); 

            bool buildKDtree();
            bool generateSamplePoints();
            bool isObstacleFree(const Node2d &node_) const;     
            
            bool generateEdges(Node2d a_, const Node2d b_);

            bool generateEdges(Node3d a_ , Node3d b_);

            bool isReachable();
            bool connectConfigurationToRobot(   geometry_msgs::Pose rp_, geometry_msgs::Pose cp_, \
                                                const std::string rp_topic_ = "rp_", const std::string cp_topic_ = "cp_", 
                                                const std::string sc_topic_ = "sc_");
            
            geometry_msgs::PoseArray generateSteeringCurve(geometry_msgs::Pose robot_pose_, const float R_);
            void generateSteeringCurveFamily(geometry_msgs::Pose robot_pose_);
        
        
            //==== variables =====
            
            Visualize visualize_;
            bool map_set_;
            nav_msgs::OccupancyGridConstPtr grid_;

            //** planner tuning params
            int N_ ;
            
            int sr_; // neighbour search radius

            //core planner vars
            std::vector<Node2d> nodes2d_;
            std::vector<geometry_msgs::Pose> steering_curve_family_poses_;

            kdTreePtr kdTree_;
            kdTree::pointVec points2d_;  //list of (x,y) GRID points  to be inserted into kd-tree

            //G[p1][theta1][p2][theta2] ==> implies that an edge exist  between 
            //std::vector<


            //** ROS members
            ros::NodeHandle nh_;
            ros::Subscriber map_sub_;


            
           
        
    };


};

#endif