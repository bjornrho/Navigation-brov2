cmake_minimum_required(VERSION 3.8)
project(odometry)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-std=c++14 -Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Boost REQUIRED)

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

add_executable(talker src/publisher_member_function.cpp)
ament_target_dependencies(talker rclcpp std_msgs Boost)

add_executable(listener src/subscriber_member_function.cpp)
ament_target_dependencies(listener rclcpp std_msgs)

# add_executable(odometry_example src/OdometryExample.cpp)
# ament_target_dependencies(odometry_example GTSAM)
# target_link_libraries(odometry_example gtsam)
# 
# add_executable(odom_node src/odom.cpp src/odom_node.cpp)
# ament_target_dependencies(odom_node nav_msgs geometry_msgs sensor_msgs tf2_ros tf2_eigen tf2_geometry_msgs)


# Custom messages
find_package(rosidl_default_generators REQUIRED)





install(TARGETS
  talker
  listener
  #odometry_example
  #odom_node
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
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DVL.msg"
  "msg/DVLBeam.msg"
  DEPENDENCIES builtin_interfaces
 )

ament_package()
