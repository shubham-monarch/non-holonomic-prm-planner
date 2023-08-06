
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

 * collisiondetectionpolygon.h => manages collision detection between convex polygons  
 * pathgenerator.h => responsible for generating a collision free path between the start and the goal points, given a roadmap
 * sampler.h => implementation of intial sampling strategies
 * roadmap.h => responsible for the roadmap generation 
 * kdtree.h => responsible for kdtree generation from the initial sampled points
 * robotmodel.h => basic robot model representation
 * steeringcurve.h => non-holonomic constraints modelling used to generate the roadmap 
 * utils.h => helper functions
 * visualization.h => constains visualization helpers to see the generated roadmap, steering curves and the final plan
## Screenshots


![Sample Generated Path](/screenshots/1.png?raw=true)
![Sample Generated Path](/screenshots/2.png?raw=true)
![Sample Generated Path](/screenshots/3.png?raw=true)


## TODO
* Create launch file
* Parametrize ackermann vehicle params