cmake_minimum_required(VERSION 3.8)
project(kraken_orchard)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(kraken_interfaces REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(row_geometry src/cloud_filter.cpp src/row_fit.cpp)
target_include_directories(row_geometry PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(row_follower src/row_follower_node.cpp)
target_link_libraries(row_follower row_geometry)
ament_target_dependencies(row_follower
  geometry_msgs
  kraken_interfaces
  nav_msgs
  rclcpp
  sensor_msgs)

install(TARGETS row_geometry
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib)
install(TARGETS row_follower DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_row_fit test/test_row_fit.cpp)
  target_link_libraries(test_row_fit row_geometry)
  ament_add_gtest(test_cloud_filter test/test_cloud_filter.cpp)
  target_link_libraries(test_cloud_filter row_geometry)
endif()

ament_package()
