cmake_minimum_required(VERSION 3.8)
project(brov2_odometry)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(brov2_interfaces REQUIRED)

find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
# find_package(eigen_conversions REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)


###### GTSAM STUFF STARTS HERE

# Include GTSAM CMake tools
find_package(GTSAMCMakeTools)

# Find GTSAM components
find_package(GTSAM REQUIRED) # Uses installed package

include(GtsamMakeConfigFile)

###### GTSAM STUFF ENDS HER

add_executable(odometry_example src/OdometryExample.cpp)
ament_target_dependencies(odometry_example GTSAM)
target_link_libraries(odometry_example gtsam)

add_executable(odom_node src/odom.cpp src/odom_node.cpp)
ament_target_dependencies(odom_node nav_msgs geometry_msgs sensor_msgs tf2_ros tf2_eigen tf2_geometry_msgs)


install(TARGETS
  odometry_example
  odom_node
  DESTINATION lib/${PROJECT_NAME}
  )

install(
  DIRECTORY include/
  DESTINATION include
)


include_directories(
  include
  include/${PROJECT_NAME}
  ${CMAKE_INCLUDE_DIRECTORIES_BEFORE}
  ${colcon_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIR}
)



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()


ament_package()
