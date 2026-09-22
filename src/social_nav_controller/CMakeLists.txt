cmake_minimum_required(VERSION 3.8)
project(social_nav_controller)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

# Optimize by default: the planner loop is Eigen-heavy and far too slow at -O0.
# Callers can override with -DCMAKE_BUILD_TYPE=...
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(social_nav_core REQUIRED)
find_package(social_nav_msgs REQUIRED)

set(dependencies
  rclcpp
  rclcpp_lifecycle
  geometry_msgs
  nav_msgs
  std_msgs
  nav2_core
  nav2_costmap_2d
  nav2_util
  pluginlib
  tf2_ros
  social_nav_core
  social_nav_msgs
)

add_library(${PROJECT_NAME} SHARED
  src/social_nav_controller.cpp
  src/path_geometry.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)
# Eigen is provided transitively (PUBLIC) via social_nav_core; use only the plain
# ament_target_dependencies signature here to avoid keyword/plain mixing.
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

# Register the controller with Nav2's plugin system.
pluginlib_export_plugin_description_file(nav2_core social_nav_controller_plugin.xml)

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)
install(FILES social_nav_controller_plugin.xml DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})

if(BUILD_TESTING)
  # Style/lint deferred to CI; logic tests only here.
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(${PROJECT_NAME}_geometry_tests test/test_path_geometry.cpp)
  target_link_libraries(${PROJECT_NAME}_geometry_tests ${PROJECT_NAME} Eigen3::Eigen)

  # Proves the controller loads via pluginlib exactly as Nav2's controller server does.
  ament_add_gtest(${PROJECT_NAME}_plugin_load_test test/test_plugin_load.cpp)
  ament_target_dependencies(${PROJECT_NAME}_plugin_load_test pluginlib nav2_core)
endif()

ament_package()
