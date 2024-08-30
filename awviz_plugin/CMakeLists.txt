cmake_minimum_required(VERSION 3.14)
project(awviz_plugin VERSION 0.0.2)

# -------- default to C++17 --------
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

# -------- create compile_commands.json --------
if(NOT CMAKE_EXPORT_COMPILE_COMMANDS)
  set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
endif()

# -------- comple options --------
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# -------- find dependencies --------
find_package(ament_cmake_auto REQUIRED)
find_package(rerun_sdk REQUIRED)
ament_auto_find_build_dependencies()

# -------- link targets --------
file(GLOB_RECURSE awviz_plugin_SOURCES
  ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp
)
add_library(${PROJECT_NAME} SHARED ${awviz_plugin_SOURCES})
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME} awviz_common::awviz_common rerun_sdk)
ament_target_dependencies(
  ${PROJECT_NAME}
  rclcpp
  cv_bridge
  sensor_msgs
  autoware_perception_msgs
)

# -------- export plugin description --------
pluginlib_export_plugin_description_file(awviz_common plugin_description.xml)

# -------- export targets --------
install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Export
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(${PROJECT_NAME}Export)

# -------- for testing --------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_lint_cmake_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
