#include <cassert>
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <non-holonomic-prm-planner/vector2d.h>
#include <non-holonomic-prm-planner/collisiondetectionpolygon_astar.h>

using namespace HybridAStar;

namespace PRM{

    void insert(PolyClr, RTree_Ptr);

    void packObstacleVector(
        const polygon_publisher::Polygon::Response& srv_response,
        std::vector<PolyClr>& obstacle_vector, Color color
    );

    void generateRGBA(Color code, std_msgs::ColorRGBA& color);

    void cleanRedundantPolygons(std::vector<PolyClr>& input);


    /// @brief Finds projection of point on line with unit_vector direction through origin
    /// @param origin Origin point on line
    /// @param unit_vector Direction of line
    /// @param point_to_project Point to project onto the line
    /// @return Parameterized distance along line
    float project(Vector2D origin, Vector2D unit_vector, Vector2D point_to_project)
    {
        return unit_vector.dot(point_to_project-origin);
    }

    CollisionDetectionPolygon::CollisionDetectionPolygon() {
        initialize();
    }

    void CollisionDetectionPolygon::initialize()
    {
        ros::NodeHandle nh;
        ros::service::waitForService(polygon_service, -1);
        m_obstacles.clear();
        bool prev_error_state = error_state;
        if (!obstacle_indices.empty())
        {
            obstacle_indices.clear();
        }
        if (!geofence_index.empty())
        {
            geofence_index.clear();
        }
        polygon_publisher::Polygon poly_srv;
        poly_srv.request.color = "white";
        ros::service::call(polygon_service, poly_srv);
        std::vector<PolyClr> white_polygons;
        packObstacleVector(poly_srv.response, white_polygons, Color::white);
        for (auto it = white_polygons.begin(); it < white_polygons.end(); it++)
        {
            if (bg::area(*(it->polygon.get())) < 10.0)
            {
                it = white_polygons.erase(it);
            }
        }
        bool valid_geofence = true;
        if (white_polygons.empty())
        {
            valid_geofence = false;
        }

        if (valid_geofence)
        {
            packIndex(white_polygons, geofence_index);
            this->error_state=false;
        } else
        {
            this->error_state = true;
            ROS_WARN("No Valid Geofence");
            if (error_state != prev_error_state)
            {
                publishErrorState();
                prev_error_state = this->error_state;
            }
        }

        ROS_INFO("white_polygons: %d", white_polygons.size());
        
        for (auto geofence : white_polygons)
        {
            obstacle_indices.emplace(std::make_pair(geofence.name,std::make_shared<RTree>()));
        }

        //Get Obstacle Polygons
        
        poly_srv.request.color = "blue";
        ros::service::call(polygon_service, poly_srv);
        std::vector<PolyClr> blue_obstacles;
        packObstacleVector(poly_srv.response, blue_obstacles, Color::blue);
        //cleanRedundantPolygons(blue_obstacles);
        for (auto poly : blue_obstacles)
        {
            m_obstacles.emplace_back(poly);
        }

        poly_srv.request.color = "green";
        ros::service::call(polygon_service, poly_srv);
        std::vector<PolyClr> green_obstacles;
        packObstacleVector(poly_srv.response,green_obstacles, Color::green);
        cleanRedundantPolygons(green_obstacles);
        
        //std::vector<PolyClr> obstacles = blue_obstacles;
        for (auto obstacle : green_obstacles)
        {
            m_obstacles.emplace_back(obstacle);
        }
        std::vector<std::vector<PolyClr>> geofence_obstacle_sets;
        for (auto obstacle : m_obstacles) {
            // See what geofences this polygon intersects with
            std::vector<Value> result;
            bgi::query(geofence_index,bgi::intersects(*(obstacle.polygon.get())), std::back_inserter(result));
            for (auto potential_intersect : result)
            {
                PolyClr const& geofence = std::get<1>(potential_intersect);
                if (bg::intersects(*(obstacle.polygon.get()),*(geofence.polygon.get())))
                {
                    RTree_Ptr current_index = obstacle_indices[geofence.name];
                    insert(obstacle, current_index);
                }
            }
        }
    // TODO: Check for empty geofences and report them by name to ROS log?
    /* 	if (!obstacles.empty() && valid_geofence)
        {
            std::vector<Value> result;
            obstacle_index.query(bgi::intersects(geofence),std::back_inserter(result));
            if (result.empty())
            {
                this->error_state = true;
                ROS_WARN("Geofence Does Not Contain Obstacles");
                if (prev_error_state != error_state)
                {
                    publishErrorState();
                    prev_error_state = this->error_state;
                }
            } else
            {
                this->error_state = false;
            }
        } */
        publishAllPolygons(false);
        if (prev_error_state != this->error_state)
        {
            publishErrorState();
        }
    }

    bool CollisionDetectionPolygon::isConfigurationFree(const std::vector<float>& obb) const {
        Polygon_t polygon;
        std::vector<Value> result;
        std::vector<Point_t> pts;
        for (size_t i=0; i<obb.size(); i+=2) {
            pts.emplace_back(obb[i],obb[i+1]);
        }
        polygon.outer().assign(pts.begin(),pts.end());
        if (!bg::within(polygon,*(current_geofence.get()))) {
            return false;
        }
        current_index->query(bgi::intersects(polygon), std::back_inserter(result));
        if (!result.empty()) {
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
        packObstacleMessage(obstacles,geofence_index);
        packObstacleMessage(obstacles,m_obstacles);
        geometry_msgs::PolygonStamped empty_geofence;
        empty_geofence.header.frame_id="map";
        empty_geofence.header.stamp = ros::Time::now();
        geofence_pub.publish(empty_geofence);
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
    //#endif //!DEBUG

    bool CollisionDetectionPolygon::carveRunway(const geometry_msgs::Pose& p1, const geometry_msgs::Pose& p2, bool start) {
        tf2::Quaternion q1(
            p1.orientation.x,
            p1.orientation.y,
            p1.orientation.z,
            p1.orientation.w
        );
        tf2::Quaternion q2(
            p2.orientation.x,
            p2.orientation.y,
            p2.orientation.z,
            p2.orientation.w
        );
        tf2::Matrix3x3 m1(q1), m2(q2);
        double roll1, roll2, pitch1, pitch2, yaw1, yaw2;
        m1.getRPY(roll1,pitch1,yaw1);
        m2.getRPY(roll2,pitch2,yaw2);
        Point_t bg_p1(p1.position.x,p1.position.y), bg_p2(p2.position.x,p2.position.y);
        PolyPtr poly_p1 = findClosestPoly(bg_p1, Color::green);
        PolyPtr poly_p2 = findClosestPoly(bg_p2, Color::green);
        //Polygon_t poly1 = *poly_p1.get();
        //Polygon_t poly2 = *poly_p2.get();
        //assert(poly_p1.get()==poly_p2.get()); //What do we do when points are near to different polygons
        size_t i_ptr_array = start?0:1;
        if (runway_m[i_ptr_array?0:1].get()==poly_p1.get())
        {
            //Start and Goal are in Same Polygon
            runway_o[i_ptr_array] = runway_o[i_ptr_array?0:1];
        } else
        {
            runway_o[i_ptr_array] = std::make_shared<Polygon_t>(*poly_p1);
        }
        runway_m[i_ptr_array] = poly_p1;
        
        // Naive Approach, just make a quadrilateral of fixed size and take the difference green_poly-quad
        // Ok this approach fails when runway points are heavily offset. This will result in a heavily skewed trapezoid
        // Want a rectangle, so test each corner
        // If corner isn't @ 90 deg (dot product)
        // find projection of vector from next corner to this one on the vector from next next corner to next corner. a.proj(b) = a.dot(b)/b.l2norm()
        // new point = next corner + proj^*vector from next next to next
        // Should need to correct the next corner as well, but for a quadrilateral only two points need to be moved (with this strategy we should only correct accute corners)
        // Other options, rotate and find AABB, rotate back (that sounds a lot simpler)
        float l_r = 6.0;
        Vector2D Vr1,Vr2,Vr3,Vr4;
        Vector2D Vp1 = Vector2D(p1.position.x,p1.position.y);
        Vector2D Vp2 = Vector2D(p2.position.x,p2.position.y);
        Vector2D unit_p1 = Vector2D(std::cos(yaw1),std::sin(yaw1));
        Vector2D unit_p2 = Vector2D(std::cos(yaw2),std::sin(yaw2));
        Vector2D p1_p = Vp1+l_r*unit_p1;
        Vector2D p1_m = Vp1-l_r*unit_p1;
        Vector2D p2_p = Vp2+l_r*unit_p2;
        Vector2D p2_m = Vp2-l_r*unit_p2;
        float t1 = project(Vp1,unit_p1,p2_m); 
        float t2 = project(Vp2,unit_p2,p1_m);
        if (abs(t1) > abs(t2))
        {
            // t1 is parameterized distance (by unit vectors) along p1 line, t2 is likewise along the line through p2
            // We want to take the furthest corner of the quadrilateral to capture maximum possible row area
            // Thus check which parameter is larger (absolute so we don't have to worry about what direction it's point)
            Vr1 = p2_m;
            Vr2 = Vp1+t1*unit_p1;
        } else
        {
            Vr1 = Vp2+t2*unit_p2;
            Vr2 = p1_m;
        }
        t1 = project(Vp1,unit_p1,p2_p); 
        t2 = project(Vp2,unit_p2,p1_p);
        if (abs(t1) > abs(t2))
        {
            Vr4 = p2_p;
            Vr3 = Vp1+t1*unit_p1;
        } else
        {
            Vr4 = Vp2+t2*unit_p2;
            Vr3 = p1_p;
        }
        
        Point_t r1( Vr1.getX(),
                            Vr1.getY());
        Point_t r2( Vr2.getX(),
                                Vr2.getY());
        Point_t r3( Vr3.getX(),
                                Vr3.getY());
        Point_t r4( Vr4.getX(),
                                Vr4.getY());
        Polygon_t runway;
        bg::append(runway.outer(),r1);
        bg::append(runway.outer(),r2);
        bg::append(runway.outer(),r3);
        bg::append(runway.outer(),r4);
        bg::correct(runway);
        std::vector<Polygon_t> output;
        bg::difference(*runway_m[i_ptr_array],runway,output);
        size_t i_max = 0;
        float area, max_area=0.f;
        assert(!output.empty());
        for (size_t i=0;i<output.size();i++)
        {
            area = bg::area(output[i]);
            if (area>max_area)
            {
                max_area=area;
                i_max=i;
            }
        }
        *runway_m[i_ptr_array].get()=output[i_max];
        //std::swap(runway_o[i_ptr_array],runway_m[i_ptr_array]);
        publishCurrentPolygons();
        if (start)
        {
            start_modified=true;
        } else
        {
            goal_modified=true;
        }
        return true;
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

    void CollisionDetectionPolygon::repairPolygons()
    {
        if (start_modified)
        {
            *runway_m[0] = *runway_o[0];
            runway_m[0] = PolyPtr(nullptr);
            runway_o[0] = PolyPtr(nullptr);
            start_modified=false;
        }
        if (goal_modified)
        {
            *runway_m[1] = *runway_o[1];
            runway_m[1] = PolyPtr(nullptr);
            runway_o[1] = PolyPtr(nullptr);
            goal_modified=false;
        }
        publishAllPolygons(false);
    }

    bool CollisionDetectionPolygon::selectCurrentIndex(Point_t p1, Point_t p2)
    {
        /*
            Typically p1 and p2 will be start and goal points
            Returns whether geofence are set succesfully, false otherwise
        */
        std::vector<Value> results;
        geofence_index.query(
            bgi::contains(p1),
            std::back_inserter(results)
        );
        
        if (results.empty())
        {
            return false;
        }

        std::vector<PolyClr> geofences_1;
        for (auto result : results)
        {
            if (bg::within(p1, *(std::get<1>(result).polygon.get())))
            {
                geofences_1.push_back(std::get<1>(result));
            }
        }

        if (geofences_1.empty())
        {
            return false;
        }

        results.clear();
        geofence_index.query(
            bgi::contains(p2),
            std::back_inserter(results)
        );
        
        if (results.empty())
        {
            return false;
        }

        std::vector<PolyClr> geofences_2;
        for (auto result : results)
        {
            if (bg::within(p2, *(std::get<1>(result).polygon.get())))
            {
                geofences_2.push_back(std::get<1>(result));
            }
        }

        if (geofences_2.empty())
        {
            return false;
        }

        std::string name;
        for (PolyClr poly1 : geofences_1)
        {
            for (PolyClr poly2 : geofences_2)
            {
                //TODO: Ew quadratic runtime
                if (poly1.name.compare(poly2.name) == 0)
                {
                    PolyClr match = poly1;
                    if (current_geofence)
                    {
                        if (bg::area(*(match.polygon.get())) > bg::area(*(current_geofence.get())))
                        {
                            current_geofence = match.polygon;
                            name = match.name;
                        }
                    } else
                    {
                        current_geofence = match.polygon;
                        name = match.name;
                    }
                }
            }
        }
        
        if (!current_geofence)
        {
            return false;
        }

        current_index = obstacle_indices[name];
        publishCurrentPolygons();
        return true;
    }

    void CollisionDetectionPolygon::clearCurrentIndex()
    {
        current_geofence = nullptr;
        current_index = nullptr;
        publishAllPolygons(false);
    }

    void CollisionDetectionPolygon::publishErrorState() const
    {
        std_msgs::Bool error_msg;
        error_msg.data = error_state;
        error_pub.publish(error_msg);
    }

    void cleanRedundantPolygons(std::vector<PolyClr>& input)
    {
        // Currently this will just take the larger of two polygons
        // Use only for cleaning redundant vineblocks
        auto it1 = input.begin();
        bool it_advanced=false;
        while (it1!=input.end())
        {
            float area1=bg::area(*(it1->polygon));
            auto it2=it1;
            it2++;
            while (it2!=input.end())
            {
                if (bg::intersects(*(it1->polygon),*(it2->polygon)))
                {
                    if (area1>bg::area(*(it2->polygon)))
                    {
                        it2=input.erase(it2);
                    } else
                    {
                        it1=input.erase(it1);
                        it_advanced=true;
                        break;
                    }
                } else
                {
                    it2++;
                }
            }
            if (!it_advanced)
            {
                it1++;
            } else
            {
                it_advanced=false;
            }
        }
    }

    void insert(PolyClr poly, RTree_Ptr index)
    {
        AABB box = bg::return_envelope<AABB>(*poly.polygon);
        index->insert(std::make_pair(box, poly));
    }

    void packObstacleVector(
        const polygon_publisher::Polygon::Response& srv_response,
        std::vector<PolyClr>& obstacle_vector, Color color
    ) {
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
                obstacle_vector.push_back(PolyClr(polygon, poly.header.frame_id, color));
            } else
            {
                ROS_WARN("Invalid Obstacle/Vineblock Polygons");
            }
        }
    }

    void generateRGBA(Color code, std_msgs::ColorRGBA& color) {
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

