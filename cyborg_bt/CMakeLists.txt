cmake_minimum_required(VERSION 2.8.3)
project(cyborg_bt)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    message_generation
)

add_message_files(
    DIRECTORY msg
    FILES
    BehaviorTreeNodes.msg
)

add_service_files(
    DIRECTORY srv
    FILES
    BehaviorTree.srv
)

catkin_python_setup()

generate_messages(
    DEPENDENCIES
    std_msgs
)

catkin_package()

catkin_install_python(
    PROGRAMS
    src/main.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
