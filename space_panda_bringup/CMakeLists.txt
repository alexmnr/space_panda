cmake_minimum_required(VERSION 3.5)
project(space_panda_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch srdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
