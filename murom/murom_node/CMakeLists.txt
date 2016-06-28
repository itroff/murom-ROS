cmake_minimum_required(VERSION 2.8.3)
project(murom_node)

find_package(catkin REQUIRED COMPONENTS nav_msgs sensor_msgs geometry_msgs kobuki_msgs roscpp nodelet tf)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES murom_node murom_nodelet
   CATKIN_DEPENDS nav_msgs sensor_msgs geometry_msgs kobuki_msgs roscpp nodelet tf
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(murom_node src/library/murom_ros.cpp
                            src/library/murom.cpp
)
target_link_libraries(murom_node ${catkin_LIBRARIES})
add_dependencies(murom_node kobuki_msgs_gencpp)

add_library(murom_nodelet src/nodelet/murom_nodelet.cpp)
target_link_libraries(murom_nodelet murom_node)


install(TARGETS murom_node
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(TARGETS murom_nodelet
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)


install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(DIRECTORY plugins
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY param
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)




