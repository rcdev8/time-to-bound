cmake_minimum_required(VERSION 3.0.2)
project(time_to_bound)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  pinocchio-interface
  roscpp
  roslib
  sensor_msgs
)

find_package(Eigen3 REQUIRED)


###################################
## catkin specific configuration ##
###################################

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES time_to_bound
#  CATKIN_DEPENDS geometry_msgs pinocchio-interface roscpp
#  DEPENDS system_lib
)

###########
## Build ##
###########
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}/time_to_bound.cpp
)

# # Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_executable(${PROJECT_NAME}_node src/${PROJECT_NAME}/time_to_bound_node.cpp)
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} ${PROJECT_NAME})
