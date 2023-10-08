#ifndef DS_H
#define DS_H

#include <boost/functional/hash.hpp>
#include <ros/ros.h>

#include <Eigen/Dense>
#include <eigen3/Eigen/Core>

#include <non-holonomic-prm-planner/constants.h>    
#include <non-holonomic-prm-planner/utils.h>


#include <unordered_map>

namespace PRM
{   

   // struct Node3d;  //forward declaration
    
    typedef Eigen::Matrix3f Mat3f;
    typedef Eigen::Vector2f Vec2f;
    typedef Eigen::Vector3f Vec3f;

    struct Node3d;
    
    typedef std::shared_ptr<Node3d> NodePtr_;
        
    typedef long long int ll;

    struct Edge;

    struct Point
    {
        float x, y; 

        Point() = default;
        Point(const float x_, const float y_): x(x_), y(y_) {}  
    };
    
    struct Node2d 
    {

        const float x_, y_; 
    
        Node2d(const float  x, const float y) : x_(x), y_(y){}
        
        ///Node2d(): x_(0.f), y_(0.f) {}

        bool operator==(const Node2d& other) const 
        {
            return x_ == other.x_ && y_ == other.y_;
        }    
    };

    struct Node2dHash 
    {
        std::size_t operator()(const Node2d& obj) const {
            std::size_t seed = 0;
            boost::hash_combine(seed, obj.x_);
            boost::hash_combine(seed, obj.y_);
            return seed;
        }
    };


    struct hashing_func {
        
        unsigned long operator()(const Vec3f& key) const {
            std::size_t seed = 0;
            boost::hash_combine(seed, key[0]);
            boost::hash_combine(seed, key[1]);
            boost::hash_combine(seed, key[2]);
            return seed ;
        }

    };

struct key_equal_fn {
    
        bool operator()(const Vec3f& t1, const Vec3f& t2) const {
            
            return((t1[0] == t2[0]) && (t1[1] == t2[1]) && (t1[2] == t2[2]));

        }
    
    };


    struct Edge
    {
        
        explicit Edge(NodePtr_ node, const float dc, const float ac):  node_(node), \
                                                                            dc_(dc),    \
                                                                            ac_(ac),    \
                                                                            tc_(ac_ + dc_)
                                                                                            
        {}

        Edge() {};

        /*void print() const
        {

            ROS_WARN("========================== EDGE =============================="); 

            ROS_INFO("destination node ===> (%f,%f,%d,%f)", node_->x_, node_->y_, node_->theta_idx_, node_->theta_);
            ROS_INFO("(dc_, ac_, tc_) ==> (%f,%f,%f)", dc_, ac_, tc_);
            ROS_INFO("========================== EDGE =============================="); 

        }*/
        
        float dc_;    //distance cost 
        float ac_ ;   //angular cost
        float tc_ ;   // total cost 

                
        NodePtr_ node_; //destination node

        
    };


    struct Node3d
    {
        

        float x_, y_;  //world co-ordinates  
        int theta_idx_;  // theta_ = (theta_) 
        float theta_;    //world heading
        float cost_;

        std::shared_ptr<Node3d> parent_;   
        
       std::shared_ptr<std::unordered_map<Vec3f, Edge, hashing_func, key_equal_fn> > edges_;

      //  std::shared_ptr<std::vector<Edge> > edges_;  //pointer to node edges

        //Node3d():x_(-1.0), y_(1.f), theta_idx_(0), theta_(-1){};
    
        Node3d(const float x, const float y, const int idx_):   
                                        x_(x), y_(y), \
                                        theta_idx_(idx_), \
                                        theta_(1.f * theta_idx_ * Constants::Planner::theta_sep_ ),
                                        cost_(std::numeric_limits<float>::max()),
                                        parent_(nullptr),
                                        edges_(std::make_shared<std::unordered_map<Vec3f, Edge, hashing_func, key_equal_fn> >())
        {
            
        }

        Node3d():   parent_(nullptr), \
                    edges_(std::make_shared<std::unordered_map<Vec3f, Edge, hashing_func, key_equal_fn> >()),
                     cost_(std::numeric_limits<float>::max())
        {

           // ROS_WARN("default constructor triggered!");

        }

        bool operator<(const Node3d& p2) const {
            // Compare based on the distance from the origin (0,0,0)
            //double dist1 = p1.x * p1.x + p1.y * p1.y + p1.z * p1.z;
            //double dist2 = p2.x * p2.x + p2.y * p2.y + p2.z * p2.z;
            return cost_ > p2.cost_; // Greater-than comparison for min heap
        }

        bool operator==(const Node3d& other_) const 
        {
            return x_ == other_.x_ && y_ == other_.y_ && theta_idx_ == other_.theta_idx_;
        }       

        
        bool addEdge(const Vec3f &key_, std::shared_ptr<Edge> &e_)
        {      
            
            if(edges_->count(key_) > 0)
            {
                ROS_ERROR("edge is already present ==> something is wrong!");
            }
            else
            {
                edges_->insert({key_, *e_});
            }
            
            return true; 
        }

        /*void print() const
        {
            ROS_WARN("========== NODE ============================");
            ROS_INFO("Node3d ==> (%f,%f,%d,%f)", x_, y_, theta_idx_, theta_ * 180.f / M_PI);
            ROS_INFO("Edges.size(): %d", edges_->size());
            ROS_INFO("=============================================");
                
        }*/
      
    };

    struct CompareNode3dPointers {
        bool operator()(const std::shared_ptr<Node3d> a, const std::shared_ptr<Node3d> b) const {
            return a->cost_> b->cost_; // Change to '<' for min heap
        }
    };

    

};


#endif