cmake_minimum_required(VERSION 3.8)
project(wall_e_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(mega_bridge src/mega_bridge.cpp)
ament_target_dependencies(mega_bridge rclcpp geometry_msgs std_msgs)

add_executable(uno_bridge src/uno_bridge.cpp)
ament_target_dependencies(uno_bridge rclcpp geometry_msgs std_msgs)


add_executable(state_machine src/state_machine.cpp)
ament_target_dependencies(state_machine rclcpp geometry_msgs std_msgs)

add_executable(controller_node src/controller_node.cpp)
ament_target_dependencies(controller_node rclcpp geometry_msgs std_msgs)

install(TARGETS
	mega_bridge
	uno_bridge
	state_machine
	controller_node
	DESTINATION lib/${PROJECT_NAME}
)
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

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
