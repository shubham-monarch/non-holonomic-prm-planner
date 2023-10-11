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

namespace bg = boost::geometry;

typedef bg::model::d2::point_xy<double> point_t;
typedef bg::model::polygon<point_t> Polygon;
typedef bg::model::box<point_t> Box;
typedef bg::model::multi_polygon<Polygon> MultiPolygon;


namespace PRM
{

    struct Pose_
    {
        float x, y, theta; 
        Pose_(float x_, float y_, float theta_): x(x_), y(y_), theta(theta_){}
        Pose_(const geometry_msgs::PoseStamped &pose): x(pose.pose.position.x), y(pose.pose.position.y), theta(tf::getYaw(pose.pose.orientation)){}
        Pose_() = default;
    };

    struct rrt_node
    {   
        float cost_;
        std::shared_ptr<rrt_node> parent_;
        Pose_ pose_;
        //std::vector<std::shared_ptr<rrt_node>> children_;
        rrt_node() = default;
        
    };

    typedef std::shared_ptr<rrt_node> rrt_nodePtr;

    class rrt
    {

        public: 

            rrt(); 

            void addNodeToTree();

            void initialPoseCb(geometry_msgs::PoseWithCovarianceStampedConstPtr pose_);
            void goalPoseCb(geometry_msgs::PoseStampedConstPtr pose_);
            void polygonCb(geometry_msgs::PolygonStampedConstPtr polygon_);


            bool plan(const geometry_msgs::PoseStamped &start_pose_, const geometry_msgs::PoseStamped &goal_pose_);        
            void reset();
            Polygon getPolygonFromPolygonMsg(const geometry_msgs::PolygonStamped &polygon_);
            bool getNextPoint(const Polygon &polygon_, Pose_ &nxt_pose_);
            Pose_ sampleRandomPoint(const Polygon &polygon);
            bool getCost(const Pose_ &a_, const Pose_ &b_, float &cost);
            bool canConnect(const Pose_ &a_, const Pose_ &b_);
            bool connectToTree(const Pose_ &pose, rrt_nodePtr &new_node_);
            bool isGoalInVicinity(const Pose_ &pose);
            void publishTree();
            float euclidDis(const Pose_ &a_, const Pose_ &b_);
            bool correctTree(rrt_nodePtr &node_, const float sr);
            void printNode(const rrt_nodePtr &node_);


        private: 

            Polygon rrt_polygon_;
            ros::Subscriber start_pose_sub_, goal_pose_sub_;
            ros::Subscriber rrt_polygon_sub_;
            ros::NodeHandle nh_;

            bool start_pose_set_, goal_pose_set_; 
            bool polygon_set_;

            geometry_msgs::PoseStamped test_start_pose_, test_goal_pose_;
            ros::Publisher rrt_tree_pub_;
            ros::Publisher start_pose_pub_, goal_pose_pub_;

            std::vector<rrt_nodePtr> tree_;
    };



};



#endif
