#ifndef ROADMAP_H
#define ROADMAP_H


#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <non-holonomic-prm-planner/KDTree.hpp>
#include <non-holonomic-prm-planner/robot_model.h>
#include <non-holonomic-prm-planner/visualizations.h>
#include <non-holonomic-prm-planner/sampler.h>

#include <unordered_set>
#include <unordered_map>

#include <prm_planner/PRMService.h>


namespace PRM{
    
    
    typedef std::shared_ptr<kdTree::KDTree> kdTreePtr;    
    typedef kdTree::point_t kdPoint;
    typedef kdTree::pointVec kdPoints;


    //class CollisionDetectionPolygon;

    //represents a sampled point with no heading information in the GRID and not WORLD
    

    class Roadmap{

        public:

       
            Roadmap(const std::string topic_);

            
            void setMapCb(nav_msgs::OccupancyGrid::ConstPtr map_);
            void initialize();
            bool generateRoadMap();    

            static int edge_cnt_;

        private: 


            // ===== functions ====

            bool buildKDtree();
            void buildGraph();

            int generateEdges(const Node2d &a_, const Node2d &b_);
        
            bool canConnect(NodePtr_ &a_ptr_, NodePtr_&b_ptr_); 

            bool isReachable();

            NodePtr_ getNodePtr(const Node3d &node_) ;

            bool connectConfigurationToRobot(   const Node3d &rp_, const Node3d &cp_, \
                                                const std::string rp_topic_ = "rp_", const std::string cp_topic_ = "cp_", 
                                                const std::string sc_topic_ = "sc_") ;
            
            bool connectConfigurationToRobot(   geometry_msgs::Pose rp_, geometry_msgs::Pose cp_, \
                                                const std::string rp_topic_ = "rp_", const std::string cp_topic_ = "cp_", 
                                                const std::string sc_topic_ = "sc_") ;
            
      
            void processSamplePoints2D(const std::vector<Node2d> &points);

            bool getPathService(prm_planner::PRMService::Request& req, prm_planner::PRMService::Response &res);
        
           
            bool getObstacleFreePath(NodePtr_ &start_, NodePtr_ &goal_);

            std::vector<Node3d> djikstra( NodePtr_ &start_, NodePtr_ &goal_);

            nav_msgs::Path generateROSPath(const std::vector<Node3d> &path_);

            bool connectStartPoseToRoadmap( NodePtr_ &node_);
            bool connectGoalPoseToRoadmap( NodePtr_ &node_);

            bool connectNodes(NodePtr_ &a_ptr_,  NodePtr_ &b_ptr_);

            void initialPoseCb(geometry_msgs::PoseWithCovarianceStampedConstPtr pose_);
            void goalPoseCb(geometry_msgs::PoseStampedConstPtr pose_);
            void clickedPointCb(geometry_msgs::PointStampedConstPtr pose_);

            NodePtr_ getNodePtrFromPose(const geometry_msgs::Pose &pose_);


            geometry_msgs::PoseArray poseArrayFromNode2dVec(const std::vector<Node2d> &points_);
            //void samplePoints()

            //==== variables =====
            
            bool map_set_;
            nav_msgs::OccupancyGridConstPtr grid_;

            //** planner tuning params
            

            kdTreePtr kdTree_;
            kdTree::pointVec points2d_;  //list of (x,y) GRID points  to be inserted into kd-tree

            //G[p1][theta1][p2][theta2] ==> implies that an edge exist  between 
            //std::vector<


            //** ROS members
            ros::NodeHandle nh_;
            ros::Subscriber map_sub_;

            ros::Subscriber start_pose_sub_, goal_pose_sub_, clicked_pt_sub_;
            ros::ServiceServer prm_service_;
            
            std::unordered_map<Vec3f, std::shared_ptr<Node3d>, hashing_func, key_equal_fn> G_;
            
            ros::Publisher start_pose_pub, goal_pose_pub;
            //std::unordered_set<Vec3f, hashing_func>  vis_;
            

            Node3d sp_; //start pose
            //Node3d gp_; //goal pose

           // std::unordered_set<Edge, EdgeHash> G_;  //graph
            
          //  Visualize visualize_;
            
            //std::shared_ptr<RobotModel> robot_;

            std::shared_ptr<Sampler> sampler_;  
            std::vector<Node2d> sampledPoints2D_;

            ros::Publisher test_pub_; 
            const std::string sampling_topic_;
           
        
    };


};

#endif