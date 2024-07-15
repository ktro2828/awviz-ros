cmake_minimum_required(VERSION 3.14)
project(awviz)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# download rerun
include(FetchContent)
FetchContent_Declare(rerun_sdk URL https://github.com/rerun-io/rerun/releases/latest/download/rerun_cpp_sdk.zip)
FetchContent_MakeAvailable(rerun_sdk)

# Ensure rerun_sdk is compiled with -fPIC
set_target_properties(rerun_sdk PROPERTIES POSITION_INDEPENDENT_CODE ON)

# setup targets
file(GLOB SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
ament_auto_add_library(${PROJECT_NAME} SHARED ${SOURCES})

# setup dependencies
target_include_directories(${PROJECT_NAME} PUBLIC include)
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} rerun_sdk)
ament_target_dependencies(${PROJECT_NAME} rclcpp rclcpp_components sensor_msgs cv_bridge tf2_ros)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "awviz::RerunLoggerNode"
  EXECUTABLE ${PROJECT_NAME}_exec
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
)
