cmake_minimum_required(VERSION 2.8.3)
project(imu_to_odom)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=gnu++11 -o3)

#############
# LIBRARIES #
#############
cs_add_library(${PROJECT_NAME}
  src/imu_to_odom.cpp
)

############
# BINARIES #
############
cs_add_executable(imu_to_odom_node
  src/imu_to_odom_node.cpp
)
target_link_libraries(imu_to_odom_node ${PROJECT_NAME})

##########
# EXPORT #
##########
cs_install()
cs_export()
