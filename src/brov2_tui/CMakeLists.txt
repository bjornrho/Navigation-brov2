cmake_minimum_required(VERSION 3.8)
project(brov2_tui)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(brov2_interfaces REQUIRED)
find_package(Curses REQUIRED)


add_executable(listen_and_display src/subscribe_and_display.cpp)
ament_target_dependencies(listen_and_display rclcpp std_msgs brov2_interfaces)
target_link_libraries(listen_and_display ncurses)

install(TARGETS
  listen_and_display
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
