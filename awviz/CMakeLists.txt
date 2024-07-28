cmake_minimum_required(VERSION 3.14)
project(awviz)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# executable
add_executable(${PROJECT_NAME} src/main.cpp)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Install executable
install(TARGETS
  ${PROJECT_NAME}
  DESTINATION bin
  DESTINATION lib/${PROJECT_NAME}
)

ament_auto_package()
