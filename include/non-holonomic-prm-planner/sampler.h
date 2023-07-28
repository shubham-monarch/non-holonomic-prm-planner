#ifndef SAMPLER_H
#define SAMPLER_H

#include <unordered_set>
#include <vector>

#include <non-holonomic-prm-planner/ds.h>


namespace PRM
{

    class Sampler
    {
        public: 

            Sampler();

            std::vector<Node2d> generate2DSamplePoints();


        private: 

            std::unordered_set<Node2d, Node2dHash> sampled_points_; 
            
    };


};






#endif