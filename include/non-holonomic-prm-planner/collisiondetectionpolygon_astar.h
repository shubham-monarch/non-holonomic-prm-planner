#ifndef COLLISIONDETECTIONPOLYGON_H
#define COLLISIONDETECTIONPOLYGON_H

#include <array>
#include <string>
#include <vector>
#include <unordered_map>
#include <memory>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/geometries.hpp>
#include <boost/geometry/index/rtree.hpp>

#include <std_msgs/Bool.h>

#include <polygon_publisher/Polygon.h>
//#ifdef DEBUG
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Polygon.h>
#include <poly_array_plugin/polygon_array.h>
//#endif //!DEBUG

#include "ros/ros.h"

namespace PRM {
  enum class Color {white, blue, green};
} //!namespace HybridAStar    
namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;
using Point_t = bg::model::point<float, 2, bg::cs::cartesian>;
using AABB = bg::model::box<Point_t>;
using Linestring_t = bg::model::linestring<Point_t>;
using Polygon_t = bg::model::polygon<Point_t,true,false>;
using PolyPtr = std::shared_ptr<Polygon_t>;
struct PolyClr {
  PolyPtr polygon;
  std::string name;
	PRM::Color color;


  PolyClr(PolyPtr polygon_, std::string name_, PRM::Color color_)
  	: polygon(polygon_), name(name_), color(color_) {}
};

using Value = std::pair<AABB, PolyClr>;
using RTree = bgi::rtree<Value, bgi::rstar<3>>;
using RTree_Ptr = std::shared_ptr<RTree>;


namespace PRM {

class CollisionDetectionPolygon {
  public:
    CollisionDetectionPolygon();
    CollisionDetectionPolygon(CollisionDetectionPolygon const&) = default;
    void initialize();
    bool isConfigurationFree(const std::vector<float>& obb) const;
    bool isConfigurationFree(float x, float y) const;
    //#ifdef DEBUG
    void publishAllPolygons(bool);
    void publishCurrentPolygons();
    void packObstacleMessage(poly_array_plugin::polygon_array& obstacles, RTree const& index);
    void packObstacleMessage(poly_array_plugin::polygon_array& obstacles, std::vector<PolyClr> const& obstacle_set);
    //#endif //!DEBUG
    bool carveRunway(const geometry_msgs::Pose& p1, const geometry_msgs::Pose& p2,bool start);
    PolyPtr findClosestPoly(const Point_t, Color);
    PolyPtr findClosestPolyMod(const Point_t, Color);
    void packIndex(std::vector<PolyClr> const& polygons, RTree& index);
    void repairPolygons();
    bool selectCurrentIndex(Point_t, Point_t);
    void clearCurrentIndex();
    bool ready() const {return !error_state;}
    bool indexSelected() const {return (current_geofence && current_index);}

    void publishClosestPolygon(PolyPtr &poly);
    

  private:
    ros::NodeHandle n;
    RTree geofence_index;
    PolyPtr current_geofence = nullptr; //Current geofence
    RTree_Ptr current_index = nullptr;
    std::unordered_map<std::string, RTree_Ptr> obstacle_indices;
    std::vector<PolyClr> m_obstacles; // Convenient way for publishAllObstacles to access poly data
    const std::string polygon_service = "/grab_polygons";
    bool start_modified=false, goal_modified=false, error_state=false;
    // *_o stores original poly, *_m stores modified poly
    std::array<PolyPtr,2> runway_o, runway_m;
		void publishErrorState() const;
		ros::Publisher error_pub = n.advertise<std_msgs::Bool>("/autodrive/path_planning/polygon_error",2,true);

    ros::Publisher geofence_pub = n.advertise<geometry_msgs::PolygonStamped>("geofence",10, true);
    ros::Publisher obstacle_pub = n.advertise<poly_array_plugin::polygon_array>("obstacles",10, true);

    //=== helper functions for vine row testing
    ros::Publisher nearest_poly_pub = n.advertise<geometry_msgs::PolygonStamped>("nearest_polygon", 1, true);

    //ros::Publisher nearest_polygon_o
}; //! class CollisionDetectionPolygon

} // ! namespace HybridAStar
#endif // !COLLISIONDETECTIONPOLYGON_H
