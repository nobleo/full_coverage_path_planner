cmake_minimum_required(VERSION 3.8)
project(full_coverage_path_planner)

################################################################################
# Compile options
################################################################################

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

################################################################################
# Required packages
################################################################################

find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

################################################################################
# Add libraries
################################################################################

set(library_name ${PROJECT_NAME})

add_library(${PROJECT_NAME} SHARED
  src/common.cpp
  src/${PROJECT_NAME}.cpp
  src/spiral_stc.cpp
)

################################################################################
# Dependencies
################################################################################

set(dependencies
  geometry_msgs
  nav_msgs
  nav2_core
  nav2_costmap_2d
  nav2_msgs
  nav2_util
  pluginlib
  rclcpp
  std_msgs
  tf2_ros
)

ament_target_dependencies(${PROJECT_NAME}
  ${dependencies}
)

################################################################################
# Include
################################################################################

include_directories(
    include
#    test/include
)
add_definitions(${EIGEN3_DEFINITIONS})

################################################################################
# Export
################################################################################

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})

pluginlib_export_plugin_description_file(nav2_core fcpp_spiral_plugin.xml)

################################################################################
# Install
################################################################################

install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  )

install(DIRECTORY include/
  DESTINATION include/
)

install(FILES fcpp_spiral_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

################################################################################
# Test
################################################################################

if (BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Ignore copyright
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

#  find_package(ament_cmake_gtest REQUIRED)
#  ament_add_gtest(test_common test/src/test_common.cpp test/src/util.cpp src/common.cpp)
#  ament_add_gtest(test_spiral_stc test/src/test_spiral_stc.cpp test/src/util.cpp src/spiral_stc.cpp src/common.cpp src/${PROJECT_NAME}.cpp)
#  ament_target_dependencies(test_spiral_stc ${${PROJECT_NAME}_EXPORTED_TARGETS})

#  find_package(OpenCV)
#  ament_target_dependencies(test_spiral_stc ${OpenCV_INCLUDE_DIRS})
#  target_link_libraries(test_spiral_stc ${OpenCV_LIBRARIES})

#  ament_add_test(test/${PROJECT_NAME}/test_${PROJECT_NAME}.test)

endif()

################################################################################
# Macro for ament package
################################################################################

ament_package()
