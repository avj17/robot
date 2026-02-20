cmake_minimum_required(VERSION 3.8)
project(autonomous_docking)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/battery_manager_node.py
  scripts/navigation_manager_node.py
  scripts/docking_controller_node.py
  scripts/mission_planner_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch worlds maps config rviz
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
