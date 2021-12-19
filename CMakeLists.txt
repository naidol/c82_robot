cmake_minimum_required(VERSION 3.5)
project(c82_robot)

find_package(ament_cmake REQUIRED)
find_package(nav2_bringup REQUIRED)
find_package(navigation2 REQUIRED)

install(DIRECTORY launch
  configs
  maps
  worlds
  #robots
  components
  urdf
  DESTINATION share/${PROJECT_NAME})

ament_package()
