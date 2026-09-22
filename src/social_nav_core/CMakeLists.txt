cmake_minimum_required(VERSION 3.8)
project(social_nav_core)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

# Optimize by default: the per-cycle planner is Eigen-heavy and unusably slow at -O0.
# Callers can still override with -DCMAKE_BUILD_TYPE=...
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/kinematics.cpp
  src/social_cost.cpp
  src/trajectory_generator.cpp
  src/collision.cpp
  src/trajectory_scorer.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)
target_link_libraries(${PROJECT_NAME} PUBLIC Eigen3::Eigen)

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3)

if(BUILD_TESTING)
  # Style/lint (clang-format, cpplint, copyright) are run in CI once license
  # headers are added. The unit-test build below reports logic-test results only.
  # TODO: add SPDX headers and re-enable ament_lint_common here.
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(${PROJECT_NAME}_tests
    test/test_angles.cpp
    test/test_kinematics.cpp
    test/test_social_cost.cpp
    test/test_trajectory_generator.cpp
    test/test_collision.cpp
    test/test_trajectory_scorer.cpp
    test/test_behavior_modes.cpp
  )
  target_link_libraries(${PROJECT_NAME}_tests ${PROJECT_NAME} Eigen3::Eigen)
endif()

ament_package()
