cmake_minimum_required(VERSION 3.8)
project(brov2_dvl)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-std=c++14 -Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(brov2_interfaces REQUIRED)

find_package(nlohmann_json)


add_executable(DVLPublisher src/dvl_publisher_node_main.cpp src/dvl_publisher_node)
ament_target_dependencies(DVLPublisher rclcpp std_msgs brov2_interfaces)
target_link_libraries(DVLPublisher nlohmann_json::nlohmann_json)
target_include_directories(DVLPublisher
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)


install(TARGETS
  DVLPublisher
  DESTINATION lib/${PROJECT_NAME}
)


include_directories(
  include
  include/${PROJECT_NAME}
  ${CMAKE_INCLUDE_DIRECTORIES_BEFORE}
  ${colcon_INCLUDE_DIRS}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
