cmake_minimum_required(VERSION 3.0.2)
project(offboard)


find_package(catkin REQUIRED COMPONENTS
  rospy
)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_msgs
  mavros
  mavros_msgs
  geometry_msgs
  cv_bridge  
  image_transport   
  tf
)


catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES offboard_py
  CATKIN_DEPENDS roscpp rospy std_msgs mavros_msgs geometry_msgs mavros cv_bridge image_transport
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include/offboard/
  include
  ${catkin_INCLUDE_DIRS}
)


add_executable(offb_node src/offb_node.cpp)
target_link_libraries(offb_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(save_image_listener src/save_image_listener.cpp)
target_link_libraries(save_image_listener ${catkin_LIBRARIES})
target_link_libraries(save_image_listener ${OpenCV_LIBRARIES})

add_executable(pbvs_node src/pbvs_node.cpp src/vision_utils.cpp)
target_link_libraries(pbvs_node ${catkin_LIBRARIES})
target_link_libraries(pbvs_node ${OpenCV_LIBRARIES})
