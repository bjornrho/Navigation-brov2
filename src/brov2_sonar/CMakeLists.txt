cmake_minimum_required(VERSION 3.8)
project(brov2_sonar)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(brov2_interfaces REQUIRED)

add_executable(brov2_sonar_exe src/sonar_publisher_node.cpp src/sonar_publisher_node_main.cpp src/DSSPParser.cpp src/DVSFileWriter.cpp src/EthernetSonarAPI.cpp)
ament_target_dependencies(brov2_sonar_exe rclcpp std_msgs brov2_interfaces)
#target_include_directories(brov2_sonar_exe PUBLIC /home/bjornar/installs/EthernetSonarAPI)

install(TARGETS
  brov2_sonar_exe
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
