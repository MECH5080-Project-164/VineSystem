# Vine Robot Software

[docker_config](docker_config) contains the Docker configuration files for the Vine Robot software. The container is a custom image based on the official ROS 2 Jazzy image. It provides packages for using Pi Cameras and WiringPi GPIO tools. The container is built using the Dockerfile located in the `docker_config` directory.

[scripts](scripts) contains the scripts used in the Vine Robot software. These are:

- [vine_system_start](scripts/vine_system_start): This script is used to start the Vine Robot Docker container.
- [find_endoscope](scripts/find_endoscope): This script finds the endoscope camera and gives it's mount point. The script can also be used to modify a yaml parameter file to set the camera mount point.
