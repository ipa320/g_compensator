cmake_minimum_required(VERSION 2.8.3)
project(g_compensation)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    geometry_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
    rosparam_shortcuts
    kdl_conversions
)

catkin_python_setup()

catkin_package()

include_directories(
    include
  ${catkin_INCLUDE_DIRS}
)

add_executable(g_compensator src/g_compensation/g_compensator.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(g_compensator
  ${catkin_LIBRARIES}
)

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
catkin_install_python(PROGRAMS
  src/g_compensation/g_compensator.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
install(TARGETS g_compensator
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

