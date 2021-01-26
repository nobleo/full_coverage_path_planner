cmake_minimum_required(VERSION 3.0.2)
project(full_coverage_path_planner)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED
        COMPONENTS
            base_local_planner
            costmap_2d
            nav_core
            pluginlib
            roscpp
            roslint
            rostest
            tf
        )

include_directories(
    include
    test/include
    ${catkin_INCLUDE_DIRS}
    )
add_definitions(${EIGEN3_DEFINITIONS})

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS
        base_local_planner
        costmap_2d
        nav_core
        pluginlib
        roscpp
)

add_library(${PROJECT_NAME}
        src/common.cpp
        src/${PROJECT_NAME}.cpp
        src/spiral_stc.cpp
        )
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    )

install(TARGETS
            ${PROJECT_NAME}
       ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       )

install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(FILES fcpp_plugin.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(
  PROGRAMS
    nodes/coverage_progress
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if (CATKIN_ENABLE_TESTING)
    catkin_add_gtest(test_common test/src/test_common.cpp test/src/util.cpp src/common.cpp)

    catkin_add_gtest(test_spiral_stc test/src/test_spiral_stc.cpp test/src/util.cpp src/spiral_stc.cpp src/common.cpp src/${PROJECT_NAME}.cpp)
    add_dependencies(test_spiral_stc ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
    target_link_libraries(test_spiral_stc ${catkin_LIBRARIES})

    find_package(OpenCV)
    include_directories(${OpenCV_INCLUDE_DIRS})
    target_link_libraries(test_spiral_stc ${OpenCV_LIBRARIES})

    add_rostest(test/${PROJECT_NAME}/test_${PROJECT_NAME}.test)

endif()

roslint_cpp()
