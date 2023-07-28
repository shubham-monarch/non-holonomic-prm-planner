#ifndef PATH_GENERATOR_H
#define PATH_GENERATOR_H

#include <unordered_map>
#include <unordered_set>

#include <geometry_msgs/PoseStamped.h>

#include <non-holonomic-prm-planner/ds.h>
#include <non-holonomic-prm-planner/robot_model.h>
#include <non-holonomic-prm-planner/visualizations.h>


namespace PRM
{

    class PathGenerator{

        public: 

            //PathGenerator(); 

            static std::vector<Node3d> getShortestPath(  
                                    std::unordered_map<Vec3f, std::shared_ptr<Node3d>, hashing_func, key_equal_fn> &G_, \
                                    std::unordered_set<Vec3f, hashing_func>  &vis_, \
                                    NodePtr_ &start_, \
                                    NodePtr_ &goal_, \
                                    Visualize &visualize_);

            static bool  getCollisionFreePath(  
                                    std::unordered_map<Vec3f, std::shared_ptr<Node3d>, hashing_func, key_equal_fn> &G_, \
                                    std::unordered_set<Vec3f, hashing_func>  &vis_, \
                                    NodePtr_ &start_, NodePtr_ &goal_,\
                                    const std::shared_ptr<RobotModel> &robot_, \
                                    Visualize &Visualize_);

            static bool checkPathForCollisions(
                                    std::unordered_map<Vec3f, std::shared_ptr<Node3d>, hashing_func, key_equal_fn> &G_, \
                                    const std::vector<Node3d>&path_, \
                                    const std::shared_ptr<RobotModel> &robot_, \
                                    Visualize &visualize_);


            static bool checkEdgeForCollisions(
                                    std::unordered_map<Vec3f, std::shared_ptr<Node3d>, hashing_func, key_equal_fn> &G_, \
                                    const std::vector<geometry_msgs::PoseStamped> &path_, \
                                    const std::shared_ptr<RobotModel> &robot_,
                                    Visualize &visualize_);



        private:

            static int obb_cnt_;

                


    };









};




#endif