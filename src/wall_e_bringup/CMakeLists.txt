cmake_minimum_required(VERSION 3.8)
project(wall_e_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(mega_node src/mega_node.cpp)
ament_target_dependencies(mega_node rclcpp geometry_msgs nav_msgs tf2 tf2_ros)

add_executable(uno_bridge src/uno_bridge.cpp)
ament_target_dependencies(uno_bridge rclcpp std_msgs)

add_executable(state_machine src/state_machine.cpp)
ament_target_dependencies(state_machine rclcpp geometry_msgs std_msgs)

add_executable(controller_node src/controller_node.cpp)
ament_target_dependencies(controller_node rclcpp geometry_msgs std_msgs)

install(TARGETS
  mega_node
  uno_bridge
  state_machine
  controller_node
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  src/llama_interpreter_node.py
  src/llama_nav_node.py
  src/fake_odom.py
  src/udp_godot_bridge.py
  src/yolo_nav_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch urdf config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()