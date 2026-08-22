cmake_minimum_required(VERSION 3.8)
project(kraken_nav)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(angles REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(kraken_interfaces REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

set(dependencies
  ament_index_cpp
  angles
  behaviortree_cpp
  geometry_msgs
  kraken_interfaces
  nav2_behavior_tree
  nav2_core
  nav2_costmap_2d
  nav2_util
  nav_msgs
  pluginlib
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  tf2
  tf2_geometry_msgs
  tf2_ros)

add_library(kraken_nav_plugins SHARED
  src/headland.cpp
  src/headland_planner.cpp
  src/arc_tracker.cpp
  src/cover_rows_navigator.cpp)
target_include_directories(kraken_nav_plugins PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(kraken_nav_plugins ${dependencies})

# The behaviour tree nodes load into the bt_navigator process, not through
# pluginlib, so they are a library of their own named in bt_navigator's
# plugin_lib_names.
add_library(kraken_cover_rows_bt_nodes SHARED
  src/cover_rows_bt_nodes.cpp)
target_include_directories(kraken_cover_rows_bt_nodes PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_definitions(kraken_cover_rows_bt_nodes PRIVATE BT_PLUGIN_EXPORT)
ament_target_dependencies(kraken_cover_rows_bt_nodes ${dependencies})

pluginlib_export_plugin_description_file(nav2_core plugins.xml)

install(TARGETS kraken_nav_plugins kraken_cover_rows_bt_nodes
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY behavior_trees config launch rviz DESTINATION share/${PROJECT_NAME})
install(FILES plugins.xml DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_headland test/test_headland.cpp)
  target_link_libraries(test_headland kraken_nav_plugins)
endif()

ament_export_include_directories(include)
ament_export_libraries(kraken_nav_plugins kraken_cover_rows_bt_nodes)
ament_export_dependencies(${dependencies})
ament_package()
