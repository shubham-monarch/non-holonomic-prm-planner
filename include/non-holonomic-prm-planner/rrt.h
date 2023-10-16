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

#include <tf/transform_datatypes.h>

#include <unordered_map>
#include <Eigen/Dense>

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
//hrrt vs ikrrt vs 
//use dubin's curve to connect in goal refgion
//control tree depth => distance = a* dis + b * del(angle) + c * rand(0,1) * depth

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
            void publishTree(const std::vector<rrt_nodePtr> &tree_);
            geometry_msgs::Pose poseFromPose_(const Pose_ pose);
            float norm(float x, float y){ return sqrt(x * x + y * y);}
            Eigen::Matrix3f getHomogeneousMatrixFromPose(const Pose_ &pose_);

            //rrt functions
            bool plan(const geometry_msgs::PoseStamped &start_pose_, const geometry_msgs::PoseStamped &goal_pose_);        
            void reset();
            bool sampleRandomPoint(const Polygon &polygon, Pose_ &pose);
            bool getClosestNode(const RTree &rtree, \
                                const PoseToNodeMap &rrt_map, \
                                const Pose_ &pose, rrt_nodePtr &closest_node);
            bool extendNode(const rrt_nodePtr &nearest_node, const Pose_ &random_pose, const float arc_len, const bool fwd);
            point_t getCircleCenter(const Pose_ &pose, const float r, const bool clockwise);
            point_t getCircleCenter(const Pose_ &pose, const float delta);
            float getTurningRadius(const float delta);
            bool getTurningRadius(const float xr, const float yr, float &r);
            float getHeadingInRobotFrame(const float xr, const float yr);
            Pose_ robotToWorldFrame(const Pose_ &robot_pose_, const Pose_ &pose);

        private: 

            Polygon rrt_polygon_;
            bool start_pose_set_, goal_pose_set_, polygon_set_; 
            geometry_msgs::PoseStamped test_start_pose_, test_goal_pose_;
            ros::Publisher rrt_tree_pub_;
            ros::Publisher start_pose_pub_, goal_pose_pub_;
            ros::Subscriber start_pose_sub_, goal_pose_sub_;
            ros::Subscriber rrt_polygon_sub_;
            ros::NodeHandle nh_;

            std::vector<rrt_nodePtr> start_rrt_, goal_rrt_; 
            RTree start_rtree_, goal_rtree_;  //rtree for start_rrt and goal_rrt
            PoseToNodeMap start_rrt_map_, goal_rrt_map_; //unordered map for start_rrt and goal_rrt
            //std::unordered_map<point_t, rrt_nodePtr> map_;
            ros::Publisher circle_pose_pub_;
            ros::Publisher arc_end_points_pub_, circle_centers_pub_;
    };
};

#endif
