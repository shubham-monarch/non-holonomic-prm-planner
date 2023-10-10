#ifndef RRT_H
#define RRT_H

//#include <non-holonomic-prm-planner/ds.h>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>

#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>


namespace bg = boost::geometry;

typedef bg::model::d2::point_xy<double> point_t;
typedef bg::model::polygon<point_t> Polygon;
typedef bg::model::box<point_t> Box;
typedef bg::model::multi_polygon<Polygon> MultiPolygon;


namespace PRM
{



    class rrt
    {

        public: 

            rrt(); 

            void addNodeToTree();

            void initialPoseCb(geometry_msgs::PoseWithCovarianceStampedConstPtr pose_);
            void goalPoseCb(geometry_msgs::PoseStampedConstPtr pose_);
            


        private: 

            Polygon polygon_;
            ros::Subscriber start_pose_sub_, goal_pose_sub_;
            ros::NodeHandle nh_;

    };



};



#endif
