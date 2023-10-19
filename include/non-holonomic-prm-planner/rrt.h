#ifndef RRT_H
#define RRT_H

//#include <non-holonomic-prm-planner/ds.h>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>

#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <tf/transform_datatypes.h>

#include <unordered_map>
#include <Eigen/Dense>
#include <set>

#include <prm_planner/PRMService.h>
#include <prm_planner/PRMServiceRequest.h>
#include <prm_planner/PRMServiceResponse.h>

namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;
typedef bg::model::d2::point_xy<double> point_t;
typedef bg::model::polygon<point_t> Polygon;
typedef bg::model::box<point_t> Box;
typedef bg::model::multi_polygon<Polygon> MultiPolygon;

//sampling goal with bias**
// two rrts
//vornoi bias
//goal biasing
//https://shuoli.github.io/robotics.pdf
//https://people.csail.mit.edu/teller/pubs/KaramanEtAl_ICRA_2011.pdf
//hrrt vs ikrrt vs 
//use dubin's curve to connect in goal refgion
//control tree depth => distance = a* dis + b * del(angle) + c * rand(0,1) * depth
//node deletion provision

namespace PRM
{    
    struct Pose_
    {
        float x, y, theta; 
        Pose_(float x_, float y_, float theta_): x(x_), y(y_), theta(theta_){}
        Pose_(const geometry_msgs::PoseStamped &pose): x(pose.pose.position.x), y(pose.pose.position.y), theta(tf::getYaw(pose.pose.orientation)){}
        Pose_() = default;
        bool operator==(const Pose_& other) const {
            return ((x == other.x) && (y == other.y) && (theta == other.theta));
        }   
    };

    struct rrt_node
    {   
        float cost_;
        std::shared_ptr<rrt_node> parent_;
        Pose_ pose_;
        std::vector<std::shared_ptr<rrt_node>> children_;
        rrt_node() = default;
    };

    struct pointKeyHash
    {
        std::size_t operator()(const point_t& key) const {
            std::size_t xHash = std::hash<float>{}(key.x());
            std::size_t yHash = std::hash<float>{}(key.y());
            return xHash ^ (yHash << 1);
        }
    };

    struct pointKeyEqual 
    {
        bool operator()(const point_t & key1, const point_t& key2) const {
            // Define the equality criteria for your custom key.
            return (key1.x() == key2.x()) && (key1.y() == key2.y());
        }
    };

    using rrt_nodePtr = std::shared_ptr<rrt_node>;        
    using PoseToNodeMap = std::unordered_map<point_t, rrt_nodePtr, pointKeyHash, pointKeyEqual>;
    using RTree = bgi::rtree<point_t, bgi::linear<16>> ;  
    using DMap = std::set<std::pair<float, float> >; 
    //shared ptr alias
    using PoseToNodeMapPtr = std::shared_ptr<PoseToNodeMap>;
    using RTreePtr = std::shared_ptr<RTree>;
    using DMapPtr = std::shared_ptr<std::set<std::pair<float, float> > >;
    
    //contains all the information related to a branch rrt
    struct rrt_container
    {   
        PoseToNodeMapPtr pose_to_node_map_;
        RTreePtr rtree_;
        DMapPtr dmap_;
        rrt_container()
        {
            pose_to_node_map_ = std::make_shared<PoseToNodeMap>();
            rtree_ = std::make_shared<RTree>();
            dmap_ = std::make_shared<DMap>();
        }
    };

    using rrt_containerPtr = std::shared_ptr<rrt_container>;

    class rrt
    {
        public: 

            rrt(); 

            //ros callbacks
            void initialPoseCb(geometry_msgs::PoseWithCovarianceStampedConstPtr pose_);
            void goalPoseCb(geometry_msgs::PoseStampedConstPtr pose_);
            void polygonCb(geometry_msgs::PolygonStampedConstPtr polygon_);
            
            //utility functions
            float euclidDis(const Pose_ &a, const Pose_ &b){ return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2));}
            void printNode(const rrt_nodePtr &node_);
            Polygon getPolygonFromPolygonMsg(const geometry_msgs::PolygonStamped &polygon_);
            geometry_msgs::Pose poseFromPose_(const Pose_ pose);
            float norm(float x, float y){ return sqrt(x * x + y * y);}
            Eigen::Matrix3f getHomogeneousMatrixFromPose(const Pose_ &pose_);
            bool isFree(const geometry_msgs::PoseStamped &pose);
            void publishRRTPath(const rrt_nodePtr &node);
            void publishTree(const std::vector<rrt_nodePtr> &tree_);
            
            //rrt functions
            bool plan(const geometry_msgs::PoseStamped &start_pose_, const geometry_msgs::PoseStamped &goal_pose_);       
            bool biDirectionalPlan(const geometry_msgs::PoseStamped &start_pose_, const geometry_msgs::PoseStamped &goal_pose_); 
            void reset();
            bool sampleRandomPoint(const Polygon &polygon, Pose_ &pose);
            bool sampleRandomPolygonPoint(const Polygon &polygon, Pose_ &pose);
            bool getNearestNodeToSampledPoint(const Pose_ &pose, rrt_nodePtr &closest_node);
            std::vector<Pose_> getNodeExtensions(const rrt_nodePtr &nearest_node, const float arc_len, const bool fwd);
            Pose_ getClosestNodeExtensionToGoal(const std::vector<Pose_> &poses, const Pose_ &goal_pose);
            bool addPoseToTree(const Pose_ &pose, const rrt_nodePtr &parent);
            bool getPathService(prm_planner::PRMService::Request& req, prm_planner::PRMService::Response &res);
            bool canConnect(const Pose_ &a, const Pose_ &b, const bool fwd); 
            bool deleteNode(const Pose_ &pose);
            bool canConnectToOtherTree(const Pose_ &pose, const rrt_containerPtr &other_container_, bool fwd);
            void publishStartAndGoalTree();
            void publishROSPath(const std::vector<rrt_nodePtr> &tree_);
            //setters
            //void setPoseToNodeMap(const PoseToNodeMapPtr &map){ curr_pose_to_node_map_ = map;}
            //void setRtree(const RTreePtr &rtree){ curr_rtree_ = rtree;}
            //void setDMap(const DMapPtr &dmap) {curr_dmap_ = dmap; }
            void setContainer(const rrt_containerPtr &container)
            {
                curr_pose_to_node_map_ = container->pose_to_node_map_;
                curr_rtree_ = container->rtree_; 
                curr_dmap_ = container->dmap_;
            }
            

        private: 

            Polygon rrt_polygon_;
            bool start_pose_set_, goal_pose_set_, polygon_set_; 
            geometry_msgs::PoseStamped test_start_pose_, test_goal_pose_;
            ros::Publisher rrt_tree_pub_;
            ros::Publisher start_pose_pub_, goal_pose_pub_;
            ros::Subscriber start_pose_sub_, goal_pose_sub_;
            ros::Subscriber rrt_polygon_sub_;
            ros::NodeHandle nh_;

            //std::vector<rrt_nodePtr> start_rrt_, goal_rrt_; 
            
            //rrt vars
            RTreePtr curr_rtree_;  //rtree for start_rrt and goal_rrt
            PoseToNodeMapPtr curr_pose_to_node_map_; //mapping between points in rtree and nodes in rrt 
            DMapPtr st_dmap_, go_dmap_, curr_dmap_; //mapping for deleted points
    
            //publishers and subsribers
            ros::Publisher circle_pose_pub_;
            ros::Publisher arc_end_points_pub_, circle_centers_pub_;
            ros::Publisher rrt_path_pub_;
            ros::ServiceServer rrt_service_;
            ros::Publisher closest_points_pub_;
            geometry_msgs::PoseArray rrt_tree_;
            ros::Publisher poly_centroid_pub_;

            rrt_nodePtr st_found_node_, go_found_node_;
            ros::Publisher st_tree_pub_, go_tree_pub_; 
            std::vector<rrt_nodePtr> combined_tree_;
            ros::Publisher ros_path_pub_;
            
    };  
};

#endif
