cmake_minimum_required(VERSION 2.8.3)
project(murom_node)

find_package(catkin REQUIRED COMPONENTS
    roscpp 
    hardware_interface
    controller_manager
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES icart_mini_driver
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

#include_directories(/usr/share/include)

include_directories(
    ${catkin_INCLUDE_DIRS}
)


add_executable(murom_node src/murom_node.cpp)

target_link_libraries(murom_node
    ${catkin_LIBRARIES}
)

#install(FILES
#    scripts/ypspur_coordinator_bridge
#    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts
#)

