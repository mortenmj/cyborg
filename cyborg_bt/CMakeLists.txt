cmake_minimum_required(VERSION 2.8.3)
project(cyborg_bt)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    geometry_msgs
    std_msgs
)

catkin_package()

catkin_python_setup()

catkin_install_python(
    PROGRAMS
    src/main.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
