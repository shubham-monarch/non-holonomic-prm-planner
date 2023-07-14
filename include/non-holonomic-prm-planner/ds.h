#ifndef DS_H
#define DS_H

#include <boost/functional/hash.hpp>
#include <ros/ros.h>

#include <Eigen/Dense>
#include <eigen3/Eigen/Core>

#include <non-holonomic-prm-planner/constants.h>    
#include <non-holonomic-prm-planner/utils.h>


namespace PRM
{   


    //tils::Eigen::Matrix3f getHomogeneousTransformationMatrix(const Eigen::Vector2f &translation, const float &theta) ;
        
    typedef Eigen::Matrix3f Mat3f;
    typedef Eigen::Vector2f Vec2f;
    typedef long long int ll;



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




    
   struct Node3d 
   {

         float x_, y_;  //world co-ordinates  
         int theta_idx_;  // theta_ = (theta_) 
        float theta_;    //world heading
        
        float cost_;

        Node3d():x_(-1.0), y_(1.f), theta_idx_(0), theta_(-1){};
        
        Node3d(const float x, const float y, const int idx_):x_(x), y_(y), \
                        theta_idx_(idx_), \
                        theta_(1.f * theta_idx_ * Constants::Planner::theta_sep_ )
        {

        }

        bool operator()(const Node3d& p1, const Node3d& p2) const {
            // Compare based on the distance from the origin (0,0,0)
            //double dist1 = p1.x * p1.x + p1.y * p1.y + p1.z * p1.z;
            //double dist2 = p2.x * p2.x + p2.y * p2.y + p2.z * p2.z;
            return p1.cost_ > p2.cost_; // Greater-than comparison for min heap
        }

        bool operator==(const Node3d& other_) const 
        {
            return x_ == other_.x_ && y_ == other_.y_ && theta_idx_ == other_.theta_idx_;
        }   

        /*bool operator<(const Node3d& other) const
        {
            return cost_ > other.cost_;
        }*/
        
        

        void print() const
        {
            ROS_INFO("========================");
            ROS_INFO("Node3d ==> (%f,%f,%d, %f)", x_, y_, theta_idx_, theta_);
            ROS_INFO("========================");
            
        }

    };

    class myComparator
    {
    public:
        int operator() (const Node3d& p1, const Node3d& p2)
        {
            return p1.cost_ > p2.cost_;
        }
    };


    //TODO ==> convert node2d to node2d*
    //directed edge from p1 -> p2
    struct Edge
    {

        const Node3d a_;
        const Node3d b_;


        explicit Edge(const Node3d &n1, const Node3d &n2): a_(n1) , b_(n2)
        {
            
            const float theta_a_ = 1.f * a_.theta_idx_ * Constants::Planner::theta_sep_;
            const float theta_b_ = 1.f * b_.theta_idx_ * Constants::Planner::theta_sep_;
            
            //checking if theta_idx_ and theta_are sane values 
            if(std::fabs(theta_a_ - a_.theta_) > 0.01 || std::fabs(theta_b_ - b_.theta_) > 0.01) 
            {
                ROS_ERROR("theta_a_ != a_.theta_ ==> Something is wrong!");
            }

            const Vec2f V_oa_(a_.x_, a_.y_); 
            const Vec2f V_ob_(b_.x_, b_.y_); 
            

            const Mat3f &P_oa_ = (Utils::getHomogeneousTransformationMatrix(V_oa_, theta_a_));      //pose of a in origin frame
            const Mat3f &P_ob_ = (Utils::getHomogeneousTransformationMatrix(V_ob_, theta_b_));      //pose of b in origin frame
            
            const Mat3f &P_ao_ = P_oa_.inverse();           //pose of o in a
            const Mat3f &P_ab_ = P_ao_ * P_ob_;             //pose of b in a

            const float x_dash_ = P_ab_(0,2);                                       // Δx in the frame of a 
            const float y_dash_ = P_ab_(1,2);
                                                                                    // Δy in the frame of a
            float theta_dash_  = std::atan2(P_ab_(1,0), P_ab_(0,0));                
            if(theta_dash_ < 0)
            {
                theta_dash_ += 2 * M_PI;
            }

            //ROS_WARN("theta_a_: %f theta_b: %f theta_dash_: %f", theta_a_, theta_b_, theta_dash_);
            const float r_ = Utils::getR(x_dash_, y_dash_);

            if(r_ > 0.f)
            {
                dis_cost_ = Constants::Planner::w_dis_ * r_ * theta_dash_;
                ang_cost_ = Constants::Planner::w_ang_ * theta_dash_;
               
            }
            else 
            {
                //implies y_dash is 0 
                
                if(x_dash_ >= 0)
                {   

                    ang_cost_ = 0 ; 
                    dis_cost_ = Constants::Planner::w_dis_ * x_dash_;
                } 
                else
                {
                    //implies reverse movement without turning
                    ang_cost_ = 0 ; 
                    dis_cost_ = Constants::Planner::w_rev_ * std::fabs(x_dash_);
                }
            }

            tc_ = dis_cost_ + ang_cost_;
        };


        bool operator==(const Edge& other) const 
        {
            return a_ == other.a_ && b_ == other.b_;
        }

        void print() const
        {
            
            ROS_INFO("======================  EDGE      ===================================");
            ROS_INFO("Node a => (%f,%f,%f)", a_.x_, a_.y_, 1.f * a_.theta_idx_ * Constants::Planner::theta_sep_);
            ROS_INFO("Node b => (%f,%f,%f)", b_.x_, b_.y_, 1.f * b_.theta_idx_ * Constants::Planner::theta_sep_);
            ROS_INFO("dis_cost: %f", dis_cost_); 
            ROS_INFO("ang_cost_: %f", ang_cost_); 
            ROS_INFO("tc_: %f", tc_);
            ROS_INFO("=====================================================================");
            
        }


        private: 

            float dis_cost_;     //distance traversal cost
            float ang_cost_;      //angular cost
            float tc_;          //total cost //tc_ = w_dc_ * dc_ + w_ac_ * ac_ ;
            
    };

    struct EdgeHash 
    {
        std::size_t operator()(const Edge& obj) const {
            
            std::size_t seed = 0;
            boost::hash_combine(seed, obj.a_.x_);
            boost::hash_combine(seed, obj.a_.y_);
            boost::hash_combine(seed, obj.a_.theta_);
            
            boost::hash_combine(seed, obj.b_.x_);
            boost::hash_combine(seed, obj.b_.y_);
            boost::hash_combine(seed, obj.b_.theta_);
            
            return seed;
        }
    };


};


#endif