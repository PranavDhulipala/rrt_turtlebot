cmake_minimum_required(VERSION 2.8.3)
project(rrt_turtlebot)


add_compile_options(-std=c++14)


find_package(catkin REQUIRED COMPONENTS
  actionlib
  base_local_planner
  costmap_2d
  geometry_msgs
  move_base
  nav_core
  nav_msgs
  pluginlib
  roscpp
  rospy
  std_msgs
  tf
  visualization_msgs
)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS actionlib base_local_planner costmap_2d geometry_msgs move_base nav_core nav_msgs pluginlib roscpp rospy std_msgs tf visualization_msgs)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)



 add_library(rrt_turtlebot_lib
   src/rrt.cpp
   src/kdTree.cpp
 )
 
add_dependencies(rrt_turtlebot_lib ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(rrt_turtlebot_lib ${catkin_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
	find_package(rostest REQUIRED)
	
	add_rostest_gtest(
	planner_test
	test/planner_test.launch
  test/planner_test.cpp
  src/rrt.cpp
  src/kdTree.cpp
	)
	
	target_link_libraries(
    planner_test
	${catkin_LIBRARIES} 
	)
	
	add_dependencies(
    planner_test
	${catkin_EXPORTED_TARGETS} 
	)
endif()
