# Working Notes for the Pi Cameras

## Ensuring the Cameras are being configured

Checking that the ArduCams are being found and configured correctly is possible through multiple methods.

The best method uses `libcamera-hello --list-cameras` on the base Raspberry Pi (i.e., not from in a docker container). This will output the connected cameras and also their indexes which will be useful when starting the ROS nodes to publish to the correct camera feeds.

Alternatively, it is possible to use system logs to check for camera detection messages. This can be done by running the following command:

```bash
dmesg | grep -i imx708 # this assumes that the ArduCams with camera module imx708 are connected
```

## Starting the ROS Nodes to publish the Camera Feeds

The ROS nodes for the ArduCams are based on [camera_ros](https://github.com/christianrauch/camera_ros) which is forked and modified ([here](https://github.com/MECH5080-Project-164/camera_ros)).

Each Camera connected will require a dedicated node to publish its feed. The best way to do this is with the [`start_pi_cam.sh`](../scripts/start_pi_cam.sh) script.

This takes the optional `camera_index` parameter which specifies which camera to publish the feed for. In general, with the Pi only having two CSI connectors, the indexes will be 0 and 1.

There is a script to find the connected cameras and their indexes in [`find_pi_cameras.sh`](../scripts/tooling/find_pi_cameras.sh) which is best run from outside the Docker container.

If more fine-grained control is desired, use the [`start_pi_cam.sh`](../scripts/start_pi_cam.sh) script as a starting point and go from there.

## Viewing the Camera Feed

To view the camera feed in ROS, we use the `rqt_image_view` tool. First, ensure that the camera node is running with the `start_pi_cam.sh [camera_id]` script. Then, in a separate terminal, run:

```bash
ros2 run rqt_image_view rqt_image_view
```

This will open a GUI where you can select the camera topic (e.g., `/camera/image_raw_camera_0`) to view the live feed from the camera. Ensure that the connection to the Pi supports displaying GUI apps with X11.

### Troubleshooting

It is possible that `rqt_image_view` may not work due to the display not being configured correctly. Possible reasons for this are:

1. ROS was started from a remote terminal without the `DISPLAY` environment variable set such as when using SSH from a different machine (i.e., not the Raspberry Pi).
2. The display was not correctly set up in the Docker container.

In the case of 1, you can try running the command with `DISPLAY=:0` to force it to use the local display:

```bash
DISPLAY=:0 ros2 run rqt_image_view rqt_image_view
```

Or better, use something like a VNC server to connect to the Raspberry Pi's desktop environment and then start various ROS applications from there. Alternatively, it is possible to forward X11 over SSH using:

```bash
ssh -X <user>@<raspberry_pi_ip>
```

In this case, it is important to have the Pi configured to allow X11 forwarding.

In the case of 2, a key fix seems to be ensuring that the local X server is accepting connections from the Docker container. You can do this by running the following command on the host machine before starting the Docker container:

```bash
xhost +local:docker
```

When using the `vine_system_start.sh` script, it should automatically configure the container for X11 forwarding. For good security, access should later be restricted back to the user running the Docker container:

```bash
xhost -local:docker
```
