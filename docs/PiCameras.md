# Working Notes for the Pi Cameras

## Viewing the Camera Feed

To view the camera feed in ROS, we use the `rqt_image_view` tool. First, ensure that the camera node is running with the `start_pi_cam.sh [camera_id]` script. Then, in a separate terminal, run:

```bash
ros2 run rqt_image_view rqt_image_view
```

This will open a GUI where you can select the camera topic (e.g., `/camera/image_raw_camera_0`) to view the live feed from the camera.

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
