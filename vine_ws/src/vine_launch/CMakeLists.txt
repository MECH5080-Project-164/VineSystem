cmake_minimum_required(VERSION 3.8)
project(vine_launch)

# Only need ament_cmake for installing resources; launch & launch_ros are
# runtime (Python) deps declared in package.xml, not required at configure.
find_package(ament_cmake REQUIRED)

install(DIRECTORY launch resources
  DESTINATION share/${PROJECT_NAME})

ament_package()
