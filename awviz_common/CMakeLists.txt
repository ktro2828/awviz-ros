cmake_minimum_required(VERSION 3.14)
project(awviz_common VERSION 0.0.4)

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
ament_auto_find_build_dependencies()

# -------- fetch contents --------
include(FetchContent)
FetchContent_Declare(
  rerun_sdk
  URL https://github.com/rerun-io/rerun/releases/download/0.20.0/rerun_cpp_sdk.zip
)
FetchContent_MakeAvailable(rerun_sdk)
# Ensure rerun_sdk is compiled with -fPIC
set_target_properties(rerun_sdk PROPERTIES POSITION_INDEPENDENT_CODE ON)

# -------- link targets --------
file(GLOB_RECURSE awviz_common_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.hpp
     ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
add_library(${PROJECT_NAME} SHARED ${awviz_common_SOURCES})
target_include_directories(
  ${PROJECT_NAME} PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                         $<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME} rerun_sdk yaml-cpp)
# NOTE: ament_target_dependencies ensures that all dependency include
# directories are ordered correctly when using an overlay workspace
ament_target_dependencies(${PROJECT_NAME} rclcpp pluginlib tf2 tf2_ros)

# -------- install targets --------
install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  LIBRARY DESTINATION lib
  INCLUDES
  DESTINATION include)

# -------- export targets --------
install(
  EXPORT ${PROJECT_NAME}Targets
  FILE ${PROJECT_NAME}Targets.cmake
  NAMESPACE ${PROJECT_NAME}::
  DESTINATION lib/cmake/${PROJECT_NAME})

# -------- create project configuration --------
include(CMakePackageConfigHelpers)
configure_package_config_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}Config.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
  INSTALL_DESTINATION lib/cmake/${PROJECT_NAME})
write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY AnyNewerVersion)

# -------- install project configuration --------
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
              ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
        DESTINATION lib/cmake/${PROJECT_NAME})

# -------- for testing --------
if(BUILD_TESTING)
  set(test_files
      test/transformation/test_tf_frame.cpp
      test/transformation/test_tf_tree.cpp
      test/transformation/test_entity_path_resolver.cpp
      test/transformation/test_tf_tree_updater.cpp
      test/transformation/test_transform_logger.cpp
      test/test_property.cpp)
  foreach(test_file IN LISTS test_files)
    get_filename_component(test_file_name ${test_file} NAME_WE)
    ament_auto_add_gtest(${test_file_name} ${test_file})
    target_link_libraries(${test_file_name} ${PROJECT_NAME})
  endforeach()
endif()

ament_auto_package()
