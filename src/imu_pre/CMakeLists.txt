cmake_minimum_required(VERSION 3.5)
project(imu_pre)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  std_msgs
)

find_package(Sophus REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES imu_pre
#  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp std_msgs
#  DEPENDS system_lib
)

set(Sophus_INCLUDE_DIRS "/usr/local/include/sophus")

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

include_directories(
${Sophus_INCLUDE_DIRS}
 
)

add_executable(${PROJECT_NAME}_node src/imu_pre_node.cpp)
target_link_libraries(${PROJECT_NAME}_node Sophus::Sophus)

 target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
 )



