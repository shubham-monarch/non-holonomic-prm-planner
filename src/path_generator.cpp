#include <non-holonomic-prm-planner/path_generator.h>
#include <non-holonomic-prm-planner/helpers.h>

#include <geometry_msgs/PoseArray.h>
#include <queue>


std::vector<PRM::Node3d> PRM::PathGenerator::getShortestPath(   std::unordered_map<Vec3f, std::shared_ptr<Node3d>, hashing_func, key_equal_fn> &G_, \
                                            std::unordered_set<Vec3f, hashing_func>  &vis_, \
                                            NodePtr_ &start_ptr_, \
                                            NodePtr_ &goal_ptr_)
{
    ROS_INFO("Inside djikstra function!");

    vis_.clear(); 

    geometry_msgs::PoseArray pq_path_;
    pq_path_.header.frame_id = "map" ; 
    pq_path_.header.stamp = ros::Time::now();

    std::priority_queue<Node3d> pq_; 

    
    const auto key_ = Utils::getNode3dkey(*start_ptr_);

    Node3d start_ = *start_ptr_;
    Node3d goal_ = *goal_ptr_;

    start_.parent_ = nullptr;
    start_.cost_ = 0 ;

    pq_.push(start_);
    
    bool reached_ = false; 

    Vec3f k_;

    Node3d curr_node_;

    int cnt_ = 0;
    while(ros::ok() && !pq_.empty())
    {
       
        
        //ROS_WARN("pq_.size(): %d", pq_.size());
        cnt_++; 

        //k_ = Utils::getNode3dkey(*pq_.top());

       curr_node_ = pq_.top();
        
        
        float curr_cost_ = curr_node_.cost_;

        if((curr_node_.x_ == goal_.x_) && (curr_node_.y_ == goal_.y_))
        {
            reached_ = true; 
            break;
        }        

        geometry_msgs::Pose p_; 
        p_.position.x = curr_node_.x_; 
        p_.position.y = curr_node_.y_; 
        p_.orientation = Utils::getQuatFromYaw(curr_node_.theta_); 

        pq_path_.poses.push_back(p_);

        
        //ROS_DEBUG("Printing top node ==> ");
        //curr_node_->print();

        pq_.pop();

        k_ = Utils::getNode3dkey(curr_node_);

        // === Checking if curr_node has already been visited
        if(vis_.find(k_) != vis_.end())
        {   
            //ROS_WARN("Curr node was already visited ==> CONTINUE");
            continue;
        }
        else{

            vis_.insert(k_);
        }


       // generateSteeringCurveFamily(*curr_node_, "family_" + std::to_string(cnt_));
        //visualize_.drawNodeNeighbours(curr_node_, "neighbours_" + std::to_string(cnt_));

        /*if(cnt_ > 10)
        {   
            ROS_INFO("cnt_ > 10 ==> Breaking!");
            break;
        }*/

        /*if(curr_node_->x_ == end_.x_ && curr_node_->y_ == end_.y_)
        {
            end_.parent_ = curr_node_->parent_; 
            reached_ = true; 
            break;
        }*/
        
        
        // ==== UPDATING NEIGHBOURS =========
        //int cnt_ = 0 ;
        for( auto &t : *curr_node_.edges_) 
        {   
            //ROS_INFO("insideQ!");
           // cnt_++;
            
            float ec_ = t.second.tc_;  //edge cost

            k_ = Utils::getNode3dkey(*t.second.node_);
            
            if(G_.find(k_) != G_.end())
            {   
                Node3d node_ = *G_[k_];

                if(node_.cost_ > curr_cost_ + ec_)
                {   
                    node_.parent_ = std::make_shared<Node3d>(curr_node_);
                    node_.cost_ = curr_cost_ + ec_;

                    //G_[key_] = nxt_node_;
                    pq_.push(node_);
                }
                
            }
            else
            {
                ROS_ERROR("Edge not found in the graph ===> Something is wrong!"); 
                //ROS_ERROR("")
            }

        }
    }
    
    ROS_INFO("cnt_: %d", cnt_);   

    ROS_INFO("pq_ ran for %d iterations!", cnt_);
    ROS_WARN("REACHED ==> %d", reached_);

    
    std::vector<Node3d> path_; 
    path_.clear();

    if(reached_)
    {
        //visualize_.publishT<geometry_msgs::PoseArray>("pq_path_" , pq_path_);

        ROS_WARN("======= REACHED : %d", reached_);

        
        std::shared_ptr<Node3d> idx_ = std::make_shared<Node3d>(curr_node_); 
        
        while(ros::ok() && idx_ != nullptr)
        {

            path_.push_back(*idx_); 
            idx_ = idx_->parent_;
        } 

        
        ROS_DEBUG("path_.size(): %d", path_.size());

        std::reverse(path_.begin(), path_.end());

        //return path_;
        //generateROSPath(path_);
        return path_;
    }

    else
    {

        ROS_ERROR("================================================");
        ROS_ERROR("=================CAN'T REACH GOAL ==============");
        ROS_ERROR("================================================");
        return path_;

    }
    //return reached_; 

}
