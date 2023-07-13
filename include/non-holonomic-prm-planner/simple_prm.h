#ifndef PRM_PLANNER_H
#define PRM_PLANNER_H

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>

#include <non-holonomic-prm-planner/visualizations.h>
#include <non-holonomic-prm-planner/KDTree.hpp>
#include <non-holonomic-prm-planner/constants.h>

#include <boost/functional/hash.hpp>
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
    struct Node2d {

        const float x_, y_; 
    
        explicit Node2d(const float  x, const float y) : x_(x), y_(y){}
        
        ///Node2d(): x_(0.f), y_(0.f) {}

        bool operator==(const Node2d& other) const {
            return x_ == other.x_ && y_ == other.y_;
        }    
    };

    struct Node2dHash {
        std::size_t operator()(const Node2d& obj) const {
            std::size_t seed = 0;
            boost::hash_combine(seed, obj.x_);
            boost::hash_combine(seed, obj.y_);
            return seed;
        }
    };

   struct Node3d {

        const float x_, y_;  //world co-ordinates  
        const int theta_idx_;  // theta_ = (theta_) 
        const float theta_;    //world heading
        

        explicit Node3d(const float x, const float y, const int idx_):x_(x), y_(y), \
                        theta_idx_(idx_), \
                        theta_(1.f * theta_idx_ * Constants::Planner::theta_sep_ )
        {

        }
                                                                
        bool operator==(const Node3d& other_) const {
            return x_ == other_.x_ && y_ == other_.y_ && theta_idx_ == other_.theta_idx_;
        }   
        
    };

    struct Node3dHash {
        std::size_t operator()(const Node3d& obj) const {
            std::size_t seed = 0;
            boost::hash_combine(seed, obj.x_);
            boost::hash_combine(seed, obj.y_);
            boost::hash_combine(seed, obj.theta_idx_);
            return seed;
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


            //**** core planner functions
            long long int set_N();
            float set_SR(); 

            bool buildKDtree();
            bool generateSamplePoints();
            bool isObstacleFree(const Node2d &node_) const;     
            
            bool generateEdges(Node2d a_, const Node2d b_);
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