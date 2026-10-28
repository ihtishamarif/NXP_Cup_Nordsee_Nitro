cmake_minimum_required(VERSION 3.16)
project(nxp_cup_gazebo)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch models worlds DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/generate_track.py DESTINATION lib/${PROJECT_NAME})

ament_package()
