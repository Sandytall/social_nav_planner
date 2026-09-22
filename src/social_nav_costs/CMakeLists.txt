cmake_minimum_required(VERSION 3.8)
project(social_nav_costs)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)  # §53: costmap update runs every cycle
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(social_nav_msgs REQUIRED)
find_package(social_nav_core REQUIRED)
find_package(Eigen3 REQUIRED)

set(dependencies
  rclcpp
  nav2_costmap_2d
  pluginlib
  geometry_msgs
  social_nav_msgs
  social_nav_core
)

add_library(${PROJECT_NAME} SHARED src/social_layer.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

pluginlib_export_plugin_description_file(nav2_costmap_2d social_layer_plugin.xml)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin)
install(FILES social_layer_plugin.xml DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})

ament_package()
