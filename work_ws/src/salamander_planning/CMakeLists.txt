cmake_minimum_required(VERSION 3.8)
project(salamander_planning)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(plansys2_executor REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)

# Declara o nó C++ e vincula as bibliotecas necessárias
add_executable(move_action_node src/move_action_node.cpp)
ament_target_dependencies(move_action_node
  rclcpp rclcpp_action plansys2_executor nav2_msgs tf2 tf2_geometry_msgs
)

add_executable(align_action_node src/align_action_node.cpp)
ament_target_dependencies(align_action_node rclcpp plansys2_executor std_msgs)

add_executable(get_cube_table_node src/get_cube_table_node.cpp)
ament_target_dependencies(get_cube_table_node rclcpp plansys2_executor std_msgs)

add_executable(put_cube_slot_node src/put_cube_slot_node.cpp)
ament_target_dependencies(put_cube_slot_node rclcpp plansys2_executor std_msgs)

add_executable(put_cube_table_node src/put_cube_table_node.cpp)
ament_target_dependencies(put_cube_table_node rclcpp plansys2_executor std_msgs)

add_executable(align_container_node src/align_container_node.cpp)
ament_target_dependencies(align_container_node rclcpp plansys2_executor std_msgs lifecycle_msgs)

add_executable(get_cube_slot_node src/get_cube_slot_node.cpp)
ament_target_dependencies(get_cube_slot_node rclcpp plansys2_executor std_msgs)

add_executable(put_cube_container_node src/put_cube_container_node.cpp)
ament_target_dependencies(put_cube_container_node rclcpp plansys2_executor std_msgs)

add_executable(reset_arm_node src/reset_arm_node.cpp)
ament_target_dependencies(reset_arm_node rclcpp plansys2_executor std_msgs)

add_executable(zero_position_node src/zero_position_node.cpp)
ament_target_dependencies(zero_position_node rclcpp plansys2_executor std_msgs)

add_executable(stack_cube_base_node src/stack_cube_base.cpp)
ament_target_dependencies(stack_cube_base_node rclcpp plansys2_executor std_msgs)

add_executable(stack_cube_top_node src/stack_cube_top.cpp)
ament_target_dependencies(stack_cube_top_node rclcpp plansys2_executor std_msgs)

install(TARGETS
  move_action_node
  align_action_node
  get_cube_table_node
  put_cube_slot_node
  reset_arm_node
  put_cube_table_node
  align_container_node
  put_cube_container_node
  get_cube_slot_node
  zero_position_node
  stack_cube_base_node
  stack_cube_top_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY pddl launch config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()