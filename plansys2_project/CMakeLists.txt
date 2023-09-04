cmake_minimum_required(VERSION 3.5)
project(plansys2_project)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(plansys2_msgs REQUIRED)
find_package(plansys2_executor REQUIRED)

set(CMAKE_CXX_STANDARD 17)

set(dependencies
    rclcpp
    rclcpp_action
    plansys2_msgs
    plansys2_executor
)

add_executable(drop_box_of_carrier_node src/drop_box_of_carrier.cpp)
ament_target_dependencies(drop_box_of_carrier_node ${dependencies})

add_executable(drop_content_of_box_node src/drop_content_of_box.cpp)
ament_target_dependencies(drop_content_of_box_node ${dependencies})

add_executable(move_robot_with_carrier_node src/move_robot_with_carrier.cpp)
ament_target_dependencies(move_robot_with_carrier_node ${dependencies})

add_executable(move_robot_node src/move_robot.cpp)
ament_target_dependencies(move_robot_node ${dependencies})

add_executable(put_box_on_carrier_node src/put_box_on_carrier.cpp)
ament_target_dependencies(put_box_on_carrier_node ${dependencies})

add_executable(put_content_in_box_node src/put_content_in_box.cpp)
ament_target_dependencies(put_content_in_box_node ${dependencies})

install(DIRECTORY launch pddl DESTINATION share/${PROJECT_NAME})

install(TARGETS
  drop_box_of_carrier_node
  drop_content_of_box_node
  move_robot_with_carrier_node
  move_robot_node
  put_box_on_carrier_node
  put_content_in_box_node 
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
endif()

ament_export_dependencies(${dependencies})

ament_package()
