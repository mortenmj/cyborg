cmake_minimum_required(VERSION 2.8.3)
project(cyborg_nav)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    geometry_msgs
    std_msgs
    message_generation)

add_message_files(
    DIRECTORY msg
    FILES GoalWithHeading.msg
)

add_service_files(
    DIRECTORY srv
    FILES
    AvailableGoals.srv
    ClosestGoal.srv
    DistanceToGoal.srv
)

generate_messages(
    DEPENDENCIES
    std_msgs
    geometry_msgs
)

catkin_package()

catkin_install_python(
    PROGRAMS
    src/available_goals.py
    src/closest_goal.py
    src/distance_to_goal.py
    src/load_map.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
