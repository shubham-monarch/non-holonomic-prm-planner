#ifndef VIS_H
#define VIS_H

#include <ros/ros.h>

#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>

namespace PRM{
    
    struct Node2d; 

    typedef std::shared_ptr<ros::Publisher> PubPtr;

    class Visualize {

        public: 
            
            Visualize();

            void drawCircle(const geometry_msgs::PoseStamped c_, const float r_, std::string topic_ = "knn_circle");

            template <typename T>
            void publishT(std::string topic_,  T msg, int qs_ = 10, bool latch_ = true)
            {
                
                std::shared_ptr<ros::Publisher> P_ = std::make_shared<ros::Publisher>();
                *P_ = nh_.advertise<T>(topic_, qs_, latch_);
                
                P_->publish(msg);
                
                publisher_list_.push_back(P_);

                //P_.publish(msg);

            }

            void draw2DNodes(const std::vector<Node2d> &nodes2d_, std::string topic_ = "sposes");

            void drawPoint(const float x_ , const float y_, const std::string topic = "point");
            void drawPoint(const geometry_msgs::Pose &pose_, const std::string topic = "point");
            

        private: 

            ros::NodeHandle nh_;

            std::vector<PubPtr> publisher_list_;
            
    
    };


};



#endif