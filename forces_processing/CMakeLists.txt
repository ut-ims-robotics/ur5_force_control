cmake_minimum_required(VERSION 2.8.3)
project(forces_processing)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs)


catkin_install_python(PROGRAMS
        scripts/force_reading_test.py
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/src
        )


catkin_package(
        #  INCLUDE_DIRS include
        #  LIBRARIES forces_processing
        #  CATKIN_DEPENDS other_catkin_pkg
        #  DEPENDS system_lib
)


include_directories(
        ${catkin_INCLUDE_DIRS}, include
)

#

add_library(forces_processing src/particle_filter.cpp src/particle_filter_node.cpp)
target_link_libraries(forces_processing ${catkin_LIBRARIES})

add_executable(particle_filter_node src/particle_filter_node.cpp)
target_link_libraries(particle_filter_node ${catkin_LIBRARIES} forces_processing)
add_dependencies(particle_filter_node
        {${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}
        forces_processing_gencpp)

install(FILES
        launch/force_reading_test.launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
        )