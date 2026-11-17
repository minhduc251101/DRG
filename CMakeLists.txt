cmake_minimum_required(VERSION 3.0.2)
project(drg)

find_package(catkin REQUIRED)
catkin_python_setup()          # installs the drg Python package from src/drg
catkin_package()

catkin_install_python(PROGRAMS nodes/drg_odometry_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY config launch rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
