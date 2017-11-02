cmake_minimum_required(VERSION 2.8.3)
project(cyborg_nav)

find_package(catkin REQUIRED)

catkin_package()

catkin_install_python(PROGRAMS src/load_map.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
