#include <non-holonomic-prm-planner/node3d.h>
#include <non-holonomic-prm-planner/ds.h>

bool PRM::Node3d::addEdge(const std::shared_ptr<Edge const> &e_)
{

    edges_->push_back(*e_);

    return true; 

}

void PRM::Node3d::print() const
{
    //ROS_INFO("Node3d ==> (%f,%f,%d, %f)", x_, y_, theta_idx_, theta_);
    ROS_WARN("========== NODE ============================");
    ROS_INFO("Edges.size(): %d", edges_->size());
    ROS_INFO("=============================================");
                
}