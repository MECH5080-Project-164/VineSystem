cmake_minimum_required(VERSION 3.8)
project(vine_launch)

find_package(ament_cmake REQUIRED)
find_package(launch REQUIRED)
find_package(launch_ros REQUIRED)

install(DIRECTORY
  launch
  resources
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
