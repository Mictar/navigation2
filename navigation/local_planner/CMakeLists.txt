cmake_minimum_required(VERSION 3.8)
project(local_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robot_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(convert src/convert.cpp)
ament_target_dependencies(convert rclcpp robot_msgs geometry_msgs nav_msgs tf2)

add_executable(move src/move.cpp)
ament_target_dependencies(move rclcpp robot_msgs geometry_msgs tf2)

add_executable(nav src/nav_by_rviz.cpp)
ament_target_dependencies(nav rclcpp geometry_msgs)

install(TARGETS
	convert
	move
	nav
	DESTINATION lib/${PROJECT_NAME}
	)
install(
	DIRECTORY launch DESTINATION share/${PROJECT_NAME}
	)
ament_package()
