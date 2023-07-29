#include <cassert>
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/PolygonStamped.h>

#include <non-holonomic-prm-planner/roadmap.h>


namespace PRM
{

	CollisionDetectionPolygon::CollisionDetectionPolygon()
	{
		
		ROS_WARN(" ================ CDP Constructor! =================");
		initialize();
	
	}

	geometry_msgs::Polygon polygonToPolygonMsg(const Polygon_t& polygon) {
		geometry_msgs::Polygon polygonMsg;
#include <non-holonomic-prm-planner/collisiondetectionpolygon.h>

		for (const auto& point : polygon.outer()) {
			geometry_msgs::Point32 pointMsg;
			pointMsg.x = bg::get<0>(point);
			pointMsg.y = bg::get<1>(point);
			polygonMsg.points.push_back(pointMsg);
		}

		return polygonMsg;
	}

	geometry_msgs::PolygonStamped polyClrToPolygonStampedMsg(const PolyClr& polyClr) {
		geometry_msgs::PolygonStamped polygonStampedMsg;

		// Assuming the PolyClr object has a name and a Polygon_t object (polyPtr)
		polygonStampedMsg.header.frame_id = "your_frame_id";
		polygonStampedMsg.header.stamp = ros::Time::now();
		polygonStampedMsg.polygon = polygonToPolygonMsg(*(polyClr.polygon));

		return polygonStampedMsg;
	}
		
	
	void CollisionDetectionPolygon::initialize()
	{	
		ROS_WARN("=======================");
		ROS_WARN("Inside CDP INITIALIZE!");
		ROS_WARN("=======================");
		
		ros::NodeHandle nh;
		//ros::service::waitForService(polygon_service, -1);

		m_obstacles.clear();
		obstacle_indices.clear();
		
		obstacle_indices.emplace(std::make_pair("obstacle",std::make_shared<RTree>()));

		polygon_publisher::Polygon poly_srv;		
		
		poly_srv.request.color = "blue";
		ros::service::call(polygon_service, poly_srv);
	
		std::vector<PolyClr> blue_obstacles;
		packObstacleVector(poly_srv.response, blue_obstacles, Color::blue);
		
		for (auto poly : blue_obstacles)
		{
			m_obstacles.emplace_back(poly);
		}

		ROS_WARN("m_obstacles.size() after BLUE: %d", m_obstacles.size());

		poly_srv.request.color = "green";
		ros::service::call(polygon_service, poly_srv);
		std::vector<PolyClr> green_obstacles;
		packObstacleVector(poly_srv.response,green_obstacles, Color::green);
		
		for (auto obstacle : green_obstacles)
		{
			m_obstacles.emplace_back(obstacle);
		}
	
		ROS_INFO("m_obstacles.size() after GREEN: %d", m_obstacles.size());
		
		std::string name_ = "obstacle"; 
		ROS_INFO("BEFORE ==> obstacle_indices[obstacle].size(): %d", obstacle_indices[name_]->size());

		for (auto obstacle : m_obstacles) {

			RTree_Ptr curr_idx_ = obstacle_indices["obstacle"];
			insert(obstacle, curr_idx_);
			
		}
		

		std::vector<geometry_msgs::PolygonStamped> polygonStampedMessages;

		int cnt_ = 8888;	

		RTree_Ptr curr_idx_ = obstacle_indices["obstacle"];
		
		ROS_WARN("====================================================") ;
		ROS_WARN("=============PUBLISING OBSTACLE POLYGONS============") ;
		ROS_WARN("====================================================") ;
		

		for (const auto& value : *curr_idx_) {
			
			const PolyClr& polyClr = value.second;
			geometry_msgs::PolygonStamped msg_ = polyClrToPolygonStampedMsg(polyClr);
			msg_.header.frame_id = "map"; 
			msg_.header.stamp = ros::Time::now();
			visualize_->publishT<geometry_msgs::PolygonStamped>("obstacles" + std::to_string(cnt_), msg_);
			cnt_++;
			
			//polygonStampedMessages.push_back(polygonStampedMsg);
    	
		}



		ROS_INFO("AFTER ==> obstacle_indices[obstacle].size(): %d", obstacle_indices[name_]->size());




	}

	
	bool CollisionDetectionPolygon::isConfigurationFree(const std::vector<float>& obb)  {
		Polygon_t polygon;
		std::vector<Value> result;
		std::vector<Point_t> pts;
		for (size_t i=0; i<obb.size(); i+=2) {
			pts.emplace_back(obb[i],obb[i+1]);
		}
		
		RTree_Ptr curr_idx_ = obstacle_indices["obstacle"];

		RTree tree_ = *curr_idx_;


		polygon.outer().assign(pts.begin(),pts.end());
	
			
		//RTree_Ptr curr_idx_ = obstacle_indices["obstacle"];
		
		curr_idx_->query(bgi::intersects(polygon), std::back_inserter(result));
		
		if (!result.empty()) {

			//ROS_ERROR("============RESULT NO EMPTY =================");
			for (auto potential_collision : result) {
				if (bg::intersects(polygon,*std::get<1>(potential_collision).polygon)) {
					return false;
				}
			}
		}
		return true;
	}

	bool CollisionDetectionPolygon::isConfigurationFree(float x, float y) const
	{
		Point_t pt(x,y);
		if (!bg::within(pt,*(current_geofence.get())))
		{
			return false;
		}
		std::vector<Value> result;

		

		current_index->query(bgi::intersects(pt),std::back_inserter(result));
		if (!result.empty())
		{
			for (auto potential_collision : result)
			{
				if (bg::intersects(pt,*std::get<1>(potential_collision).polygon))
				{
					return false;
				}
			}
		}
		return true;
	}

	//#ifdef DEBUG
	void CollisionDetectionPolygon::publishCurrentPolygons(void) {
		publishAllPolygons(true);
		geometry_msgs::PolygonStamped geofence_;
		poly_array_plugin::polygon_array obstacles;
		geofence_.header.frame_id="map";
		obstacles.header.frame_id="map";
		packObstacleMessage(obstacles,*(current_index.get()));
		geofence_.polygon.points.reserve(current_geofence->outer().size());
		for (auto pt : current_geofence->outer()) 
		{
			geometry_msgs::Point32 new_point;
			new_point.x = pt.get<0>();
			new_point.y = pt.get<1>();
			new_point.z = 0.f;
			geofence_.polygon.points.push_back(new_point);
		}
		geofence_.header.stamp = ros::Time::now();
		geofence_pub.publish(geofence_);
		obstacles.header.stamp = ros::Time::now();
		obstacle_pub.publish(obstacles);
	}

	void CollisionDetectionPolygon::publishAllPolygons(bool clear)
	{
		poly_array_plugin::polygon_array obstacles;
		obstacles.header.frame_id="map";
		if (clear)
		{
			// Publish empty array to clear viz
			obstacle_pub.publish(obstacles);
			return;
		}
		
		packObstacleMessage(obstacles,m_obstacles);
		obstacles.header.stamp = ros::Time::now();
		obstacle_pub.publish(obstacles);
	}

	void CollisionDetectionPolygon::packObstacleMessage(poly_array_plugin::polygon_array& obstacles, RTree const& index) {
		for (auto polygon = index.begin(); polygon!=index.end();polygon++)
		{
			geometry_msgs::Polygon new_poly;
			std_msgs::ColorRGBA color;
			poly_array_plugin::polygon new_color_poly;
			generateRGBA(std::get<1>(*polygon).color,color);
			new_poly.points.reserve(std::get<1>(*polygon).polygon->outer().size());
			for (auto pt : std::get<1>(*polygon).polygon->outer())
			{
				geometry_msgs::Point32 new_point;
				new_point.x = pt.get<0>();
				new_point.y = pt.get<1>();
				new_point.z = 0.f;
				new_poly.points.push_back(new_point);
			}
			new_color_poly.polygon = new_poly;
			new_color_poly.color = color;
			obstacles.polygons.push_back(new_color_poly);
		}
	}

	void CollisionDetectionPolygon::packObstacleMessage(poly_array_plugin::polygon_array& obstacles, std::vector<PolyClr> const& obstacle_set)
	{
		for (auto polygon = obstacle_set.begin(); polygon!=obstacle_set.end();polygon++)
		{
			geometry_msgs::Polygon new_poly;
			std_msgs::ColorRGBA color;
			poly_array_plugin::polygon new_color_poly;
			generateRGBA(polygon->color,color);
			new_poly.points.reserve(polygon->polygon->outer().size());
			for (auto pt : polygon->polygon->outer())
			{
				geometry_msgs::Point32 new_point;
				new_point.x = pt.get<0>();
				new_point.y = pt.get<1>();
				new_point.z = 0.f;
				new_poly.points.push_back(new_point);
			}
			new_color_poly.polygon = new_poly;
			new_color_poly.color = color;
			obstacles.polygons.push_back(new_color_poly);
		}
	}
	
	PolyPtr CollisionDetectionPolygon::findClosestPoly(const Point_t pt, Color color_) 
	{
		std::vector<Value> result;
		current_index->query( bgi::nearest(pt,3) //ASSUMPTION
														&& bgi::satisfies([color_](Value const& v){return std::get<1>(v).color==color_;}),
														std::back_inserter(result));
		double shortest_distance=100000000;
		//auto closest_value = obstacle_index.begin();
		size_t i = 0, i_min=0;
		for (auto value : result)//(auto value=obstacle_index.begin();value!=obstacle_index.end();value++)// : result)
		{
			// AABB means that non-cardinal polygons the closest AABB may not be the one that actually contains the closest polygon
			double distance = bg::distance(pt, *std::get<1>(value).polygon);
			if (distance < shortest_distance)
			{
				//closest_value=value;
				shortest_distance = distance;
				i_min=i;
			}
			i++;
		}
		return std::get<1>(result[i_min]).polygon;//std::get<1>(*closest_value).polygon;
	}

	void CollisionDetectionPolygon::packIndex(std::vector<PolyClr> const& polygons, RTree& index)
	{
		for (auto poly : polygons) {
			AABB box = bg::return_envelope<AABB>(*poly.polygon);
			index.insert(std::make_pair(box, poly));
		}
	}

	RTree_Ptr CollisionDetectionPolygon::selectCurrentIndex()
	{

		if((int)obstacle_indices.size() > 1)
		{	
			ROS_ERROR("======================================================");
			ROS_ERROR("obstacle_indices.size() > 1 ===> Something is wrong!!");
			ROS_ERROR("======================================================");
			return nullptr;
		}

		return obstacle_indices.begin()->second;

	
	}


	void CollisionDetectionPolygon::insert(PolyClr poly, RTree_Ptr index)
	{
		AABB box = bg::return_envelope<AABB>(*poly.polygon);
		index->insert(std::make_pair(box, poly));
	}

	void CollisionDetectionPolygon::packObstacleVector(	const polygon_publisher::Polygon::Response& srv_response, \
														std::vector<PolyClr>& obstacle_vector, Color color) 
	{
		for (auto poly : srv_response.polygon_array.obstacles) {
			if (!poly.polygon.points.empty() && !poly.header.frame_id.empty())
			{
				PolyPtr polygon(new Polygon_t);
				for (auto point : poly.polygon.points)
				{
					bg::append((*polygon).outer(), Point_t(point.x, point.y));
				}
				bg::correct(*polygon);
				if (bg::area(*(polygon.get())) < 0.0)
				{
					bg::reverse(*(polygon.get()));
				}
				//ROS_WARN("Polygon Name: %s", poly.header.frame_id.c_str());
				obstacle_vector.push_back(PolyClr(polygon, poly.header.frame_id, color));
			} else
			{
				ROS_WARN("Invalid Obstacle/Vineblock Polygons");
			}
		}
	}

	void CollisionDetectionPolygon::generateRGBA(Color code, std_msgs::ColorRGBA& color) {
		color.a = 0.7;
		switch (code){
			case Color::white:
				color.r = 1.0;
				color.g = 1.0;
				color.b = 1.0;
				return;
			case Color::green:
				color.r = 0.2;
				color.g = 0.8;
				color.b = 0.f;
				return;
			case Color::blue:
				color.r = 0.25;
				color.g = 0.f;
				color.b = 0.9;
				return;
			default:
				color.r = 0.5;
				color.g = 0.f;
				color.b = 0.9;   
		}
	}

};