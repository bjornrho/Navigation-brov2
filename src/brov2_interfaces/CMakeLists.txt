cmake_minimum_required(VERSION 3.8)
project(brov2_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Barometer.msg"
  "msg/DVL.msg"
  "msg/DVLBeam.msg"
  "msg/DVLOdom.msg"
  "msg/Reference.msg"
  "msg/Sonar.msg"
  DEPENDENCIES std_msgs geometry_msgs
 )

ament_package()
