cmake_minimum_required(VERSION 3.5)
project(plugin_application)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(polygon_base REQUIRED)

add_executable(polygon_loader
    src/polygon_loader.cpp)
ament_target_dependencies(polygon_loader
  "rclcpp"
  "pluginlib"
  "polygon_base"
  )
install(TARGETS polygon_loader
    DESTINATION lib/${PROJECT_NAME}
    )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
