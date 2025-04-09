# Vine Robot Software

[docker_config](docker_config) contains the Docker configuration files for the Vine Robot software. The container is a custom image based on the official ROS 2 Jazzy image. It provides packages for using Pi Cameras and WiringPi GPIO tools. The container is built using the Dockerfile located in the `docker_config` directory.

[scripts](scripts) contains the scripts used in the Vine Robot software. These are:

- [vine_system_start](scripts/vine_system_start): This script is used to start the Vine Robot Docker container.
- [find_endoscope](scripts/find_endoscope): This script finds the endoscope camera and gives it's mount point. The script can also be used to modify a yaml parameter file to set the camera mount point.
- [start_pi_camera](scripts/start_pi_camera): This script starts the Pi Camera. It uses the `camera_ros` package and defers in all configuration to that. The script does accept an argument for the index of the camera to start (optional). This is used when there are multiple cameras connected through CSI.

## Raspbian

Mostly, we use Raspbian as it is out of the box. We have only made a change to the `/boot/firmware/config.txt` file to enable the camera. The file was modified as such:

```bash
- camera_auto_detect=1
+ camera_auto_detect=0
# [...]
[All]
+ dtoverlay=imx708,cam0
+ dtoverlay=imx708,cam1
```

This change in necassary to work with the ArduCams. Note that for an official Pi Camera Module 3, we didn't need to make this change. The camera was detected automatically and worked out of the box.

## Docker

The Docker container is built using the Dockerfile located in the `docker_config` directory. The container is based on the official ROS 2 Jazzy image and includes packages for using Pi Cameras and WiringPi GPIO tools. The container is built using the following command:

```bash
docker build -t vine_robot .
```
