cmake_minimum_required(VERSION 3.0.2)
project(xarm_cell_viz)
find_package(catkin REQUIRED)
catkin_package()
catkin_install_python(PROGRAMS scripts/cell_poses.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
