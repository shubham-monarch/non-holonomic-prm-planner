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

    struct rrt_node
    {
        float x, y, theta;
        rrt_node(float x_, float y_, float theta_): x(x_), y(y_), theta(theta_){}
        rrt_node(const geometry_msgs::PoseStamped &p): x(p.pose.position.x), y(p.pose.position.y),theta(tf::getYaw(p.pose.orientation)){}

    };

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
            

        private: 

            Polygon rrt_polygon_;
            ros::Subscriber start_pose_sub_, goal_pose_sub_;
            ros::Subscriber rrt_polygon_sub_;
            ros::NodeHandle nh_;

            bool start_pose_set_, goal_pose_set_; 
            bool polygon_set_;

            geometry_msgs::PoseStamped test_start_pose_, test_goal_pose_;

            std::vector<rrt_node> tree_;
    };



};



#endif
