#ifndef PRM_PLANNER_H
#define PRM_PLANNER_H


#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <non-holonomic-prm-planner/visualizations.h>
#include <non-holonomic-prm-planner/KDTree.hpp>
//#include <non-holonomic-prm-planner/utils.h>
#include <non-holonomic-prm-planner/ds.h>


#include <unordered_set>
#include <unordered_map>



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

            static int edge_cnt_;

        private: 


            // ===== functions ====

            //bool generateGraph();
            bool buildKDtree();
            bool generateSamplePoints();
            bool isObstacleFree(const Node2d &node_) const;     
            
            void buildGraph();

            int generateEdges(const Node2d &a_, const Node2d &b_);
        
            //bool canConnect(const Node3d &a_ , const Node3d &b_) ;
            bool canConnect(const Node3d &a_ , const Node3d &b_, std::shared_ptr<Edge> &e_) ;

            bool isReachable();
            
            bool connectConfigurationToRobot(   const Node3d &rp_, const Node3d &cp_, \
                                                const std::string rp_topic_ = "rp_", const std::string cp_topic_ = "cp_", 
                                                const std::string sc_topic_ = "sc_") ;
            
            bool connectConfigurationToRobot(   geometry_msgs::Pose rp_, geometry_msgs::Pose cp_, \
                                                const std::string rp_topic_ = "rp_", const std::string cp_topic_ = "cp_", 
                                                const std::string sc_topic_ = "sc_") ;
            


            geometry_msgs::PoseArray generateSteeringCurve( geometry_msgs::Pose robot_pose_, const float R_, \
                                                            const bool trim_  = false, \
                                                            const float x_dash_ = -1.f);

           std::vector<geometry_msgs::PoseStamped> generateSteeringCurveTrimmed(const geometry_msgs::Pose &rp_, const geometry_msgs::Pose &cp_);

            void generateSteeringCurveFamily(const Node3d &node_, std::string topic_ = "family_");
            void generateSteeringCurveFamily(geometry_msgs::Pose robot_pose_, std::string topic_ = "family_");


            bool djikstra( Node3d &start_,  Node3d &goal_);

            nav_msgs::Path generateROSPath(const std::vector<Node3d> &path_);

            bool connectToRoadmap(const NodePtr_ &node_);

            bool connectNodes(const Node3d &a_, const Node3d &b_);

            void initialPoseCb(geometry_msgs::PoseWithCovarianceStampedConstPtr pose_);
            void goalPoseCb(geometry_msgs::PoseStampedConstPtr pose_);

            //==== variables =====
            
            Visualize visualize_;
            bool map_set_;
            nav_msgs::OccupancyGridConstPtr grid_;

            //** planner tuning params
            std::unordered_set<Node2d, Node2dHash> sampled_points_; 

            kdTreePtr kdTree_;
            kdTree::pointVec points2d_;  //list of (x,y) GRID points  to be inserted into kd-tree

            //G[p1][theta1][p2][theta2] ==> implies that an edge exist  between 
            //std::vector<


            //** ROS members
            ros::NodeHandle nh_;
            ros::Subscriber map_sub_;

            ros::Subscriber start_pose_sub_, goal_pose_sub_;
            //std::vector<Node3d> sampled_points_3d_;

            std::unordered_map<Vec3f, std::shared_ptr<Node3d>, hashing_func, key_equal_fn> G_;
            std::unordered_map<Vec3f, bool, hashing_func, key_equal_fn>  vis_;
            

            Node3d sp_; //start pose
            Node3d gp_; //goal pose

           // std::unordered_set<Edge, EdgeHash> G_;  //graph
            

            
           
        
    };


};

#endif