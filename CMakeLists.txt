cmake_minimum_required(VERSION 2.8.3)
project(prm_planner)


## C++11
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
    message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -DDEBUG")
set(CMAKE_BUILD_TYPE Release)

#add_compile_definitions(BOOST_MATH_NO_LONG_DOUBLE_MATH_FUNCTIONS)

find_package(catkin REQUIRED COMPONENTS
    message_generation
    roscpp
    rospy
    std_msgs
    tf
    nav_msgs
    geometry_msgs
    polygon_publisher
    poly_array_plugin
    autodrive_msgs
)


find_package(Eigen3 REQUIRED)

add_service_files(
    FILES
    PRMService.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
    std_msgs
    geometry_msgs
    nav_msgs
)


set(SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/roadmap.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/visualizations.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/KDTree.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/steering_curve.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/collisiondetectionpolygon_astar.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/robot_model.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/path_generator.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/sampler.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/rrt.cpp
    
    
    )

    set(HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/include/non-holonomic-prm-planner/constants.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/non-holonomic-prm-planner/roadmap.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/non-holonomic-prm-planner/utils.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/non-holonomic-prm-planner/visualizations.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/non-holonomic-prm-planner/KDTree.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/non-holonomic-prm-planner/ds.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/non-holonomic-prm-planner/helpers.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/non-holonomic-prm-planner/steering_curve.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/non-holonomic-prm-planner/collisiondetectionpolygon_astar.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/non-holonomic-prm-planner/robot_model.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/non-holonomic-prm-planner/path_generator.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/non-holonomic-prm-planner/sampler.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/non-holonomic-prm-planner/vector2d.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/non-holonomic-prm-planner/rrt.h
    
    
    
    )

add_library(PRM ${SOURCES} ${HEADERS})
add_dependencies(PRM ${catkin_EXPORTED_TARGETS})
#add_library(KDTree SHARED KDTree.cpp)
#target_include_directories(KDTree PUBLIC .)



## Declare a catkin package
catkin_package(
#  INCLUDE_DIRS
#  LIBRARIES
  CATKIN_DEPENDS
    message_runtime
    std_msgs
    geometry_msgs
    nav_msgs
    autodrive_msgs
    polygon_publisher
    poly_array_plugin
#  DEPENDS
)

## OPEN MOTION PLANNING LIBRARY
find_package(ompl REQUIRED)

if(NOT OMPL_FOUND)
    message(AUTHOR_WARNING,"Open Motion Planning Library not found")
endif(NOT OMPL_FOUND)

find_package(Boost 1.56 REQUIRED)

include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(include ${OMPL_INCLUDE_DIRS})
include_directories(include ${Boost_INCLUDE_DIRS})
include_directories(include include)

#add_executable(tf_broadcaster src/tf_broadcaster.cpp)
#target_link_libraries(tf_broadcaster ${catkin_LIBRARIES})

add_executable(non-holonomic-prm-planner src/main.cpp) # ${HEADERS} ${SOURCES})
target_link_libraries(non-holonomic-prm-planner PRM)
target_link_libraries(non-holonomic-prm-planner ${catkin_LIBRARIES})
target_link_libraries(non-holonomic-prm-planner ${OMPL_LIBRARIES})
target_link_libraries(non-holonomic-prm-planner Eigen3::Eigen)
#target_link_libraries(non-holonomic-prm-planner KDTree)


add_dependencies(non-holonomic-prm-planner ${catkin_EXPORTED_TARGETS})

#install(TARGETS ${PROJECT_NAME} tf_broadcaster
    #ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   # LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  #  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 #   )

install(DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
    )

#add_dependencies(PRM non-holonomic-prm-planner_generate_messages_cpp)
