cmake_minimum_required(VERSION 3.5)
project(polygon_plugins)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(polygon_base REQUIRED)

# include
include_directories(include)

# library
add_library(polygon_plugins SHARED
    src/polygon_plugins.cpp)
target_compile_definitions(polygon_plugins
    PRIVATE "MANIPULATOR_IK_SOLVOR_PLUGINS_BUILDING_DLL"
#    PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS"
)
ament_target_dependencies(polygon_plugins
  "pluginlib"
  "polygon_base"
)

install(TARGETS polygon_plugins
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

install(FILES plugin.xml
  DESTINATION share/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(polygon_plugins)
pluginlib_export_plugin_description_file(polygon_base plugin.xml)

ament_package()
