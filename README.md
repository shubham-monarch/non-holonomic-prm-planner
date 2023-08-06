
# non-holonomic-prm-planner

A PRM based non-holomic constraints based global planner implementation for ackermann vehicles. It is also capable of generating reverse paths. 



## Brief Overview


* Sample the C-space and collect collision free vertices
* Non-holonomic edge generation using steering curves 
* Shortest Path Search over the roadmap 

## Running 

Update constants.h file with the vehicle params
go to the ```catkin``` workspace and build the package 
```bash
    catkin build non-holonomic-prm-planner
```

Publish the obstacles polygons  
(Make sure to pass one of the files provided in the ```obstacles``` folder)   

```bash
  rosrun polygon_publisher polygon_provider_new_csv.py
```

Publish the global costmap
```bash
  rosrun global_costmap_publisher costmap_publisher_offline.py
```

Run the PRM node 
```bash
  rosrun non-holonomic-prm-planner non-holonomic-prm-planner
```
Run ```Rviz``` and pass the config file in the ```Rviz``` folder

## Headers Overview

 * [collisiondetectionpolygon.h](include/non-holonomic-prm-planner/collisiondetectionpolygon.h) => manages collision detection between convex polygons  
 * [pathgenerator.h](include/non-holonomic-prm-planner/path_generator.h) => responsible for generating a collision free path between the start and the goal points, given a roadmap
 * [sampler.h](include/non-holonomic-prm-planner/sampler.h) => implementation of intial sampling strategies
 * [roadmap.h](include/non-holonomic-prm-planner/roadmap.h) => responsible for the roadmap generation 
 * [kdtree.h](include/non-holonomic-prm-planner/KDTree.hpp) => responsible for kdtree generation from the initial sampled points
 * [robotmodel.h](include/non-holonomic-prm-planner/robot_model.h) => basic robot model representation
 * [steeringcurve.h](include/non-holonomic-prm-planner/steering_curve.h) => non-holonomic constraints modelling used to generate the roadmap 
 * [utils.h](include/non-holonomic-prm-planner/utils.h) => helper functions
 * [visualization.h](include/non-holonomic-prm-planner/visualizations.h) => constains visualization helpers to see the generated roadmap, steering curves and the final plan
## Screenshots


![Sample Generated Path](/screenshots/1.png?raw=true)
![Sample Generated Path](/screenshots/2.png?raw=true)
![Sample Generated Path](/screenshots/3.png?raw=true)


## TODO
* Create launch file
* Parametrize ackermann vehicle params