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

# Declara o nó C++ e vincula as bibliotecas necessárias
add_executable(move_action_node src/move_action_node.cpp)
ament_target_dependencies(move_action_node
  rclcpp
  rclcpp_action
  plansys2_executor
  nav2_msgs
  tf2
  tf2_geometry_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Instala o executável compilado na pasta lib do ROS 2
install(TARGETS
  move_action_node
  DESTINATION lib/${PROJECT_NAME}
)

# Exporta as pastas estáticas do projeto
install(DIRECTORY pddl launch config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()