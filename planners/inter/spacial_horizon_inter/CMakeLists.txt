cmake_minimum_required(VERSION 3.0.2)
project(spacial_horizon_inter)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -Wall")

find_package(catkin REQUIRED COMPONENTS
  mbf_costmap_core
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  std_msgs
  visualization_msgs
  tf
  actionlib
)
find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)

# this is important to find eigen path
set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})

generate_dynamic_reconfigure_options(
  cfg/SpacialHorizonInter.cfg
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES spacial_horizon_inter
  CATKIN_DEPENDS mbf_costmap_core visualization_msgs geometry_msgs nav_msgs roscpp rospy std_msgs tf actionlib
)

# ##########
# # Build ##
# ##########
include_directories(include)
include_directories(cfg)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
  ${GSTREAMER_INCLUDE_DIRS}
)
add_library(spacial_horizon_inter src/spacial_horizon_inter.cpp)

add_dependencies(spacial_horizon_inter ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(spacial_horizon_inter ${PROJECT_NAME}_gencfg)

target_link_libraries(spacial_horizon_inter
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

# ############
# # Install ##
# ############
install(TARGETS spacial_horizon_inter
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(FILES mbf_inter_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY
  cfg
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)

# ############
# # Testing ##
# ############
