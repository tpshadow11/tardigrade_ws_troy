cmake_minimum_required(VERSION 3.16)
project(tardigrade_bt)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tardigrade_interfaces REQUIRED)
# Built from source by docker/Dockerfile into /usr/local.
find_package(behaviortree_cpp REQUIRED)

# The nodes only depend on BehaviorTree.CPP, not on ROS, so tests can run
# them against a fake robot.
add_library(tardigrade_bt_nodes src/nodes.cpp)
target_include_directories(tardigrade_bt_nodes PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(tardigrade_bt_nodes PUBLIC BT::behaviortree_cpp)

add_executable(mission_runner src/mission_runner.cpp)
target_link_libraries(mission_runner tardigrade_bt_nodes)
ament_target_dependencies(mission_runner
  rclcpp ament_index_cpp geometry_msgs nav_msgs std_msgs tardigrade_interfaces)
# Groot2 live view is available when BehaviorTree.CPP was built with ZeroMQ.
if(EXISTS "/usr/local/include/behaviortree_cpp/loggers/groot2_publisher.h")
  target_compile_definitions(mission_runner PRIVATE TARDIGRADE_BT_GROOT2)
endif()

install(TARGETS tardigrade_bt_nodes mission_runner
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY trees launch DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/bt_view.py DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_trees test/test_trees.cpp)
  target_link_libraries(test_trees tardigrade_bt_nodes)
  target_compile_definitions(test_trees PRIVATE
    TREES_DIR="${CMAKE_CURRENT_SOURCE_DIR}/trees")
endif()

ament_package()
