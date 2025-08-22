# Vine Robot Software

[docker_config](docker_config) contains the Docker configuration files for the Vine Robot software. The container is a custom image based on the official ROS 2 Jazzy image. It provides packages for using Pi Cameras and WiringPi GPIO tools. The container is built using the Dockerfile located in the `docker_config` directory.

[scripts](scripts) contains the scripts used in the Vine Robot software. These are:

- [vine_system_start](scripts/vine_system_start): This script is used to start the Vine Robot Docker container.
- [find_endoscope](scripts/find_endoscope): This script finds the endoscope camera and gives it's mount point. The script can also be used to modify a yaml parameter file to set the camera mount point.
- [start_pi_camera](scripts/start_pi_camera): This script starts the Pi Camera. It uses the `camera_ros` package and defers in all configuration to that. The script does accept an argument for the index of the camera to start (optional). This is used when there are multiple cameras connected through CSI.

[vine_ws](vine_ws) contains the ROS workspace for the Vine Robot software. Further details on how to use this workspace can be found in the [`vine_ws/README.md`](vine_ws/README.md) file.

## Raspbian

Mostly, we use Raspbian as it is out of the box. We edit `/boot/firmware/config.txt` file to enable the camera and another to enable the external PCIe port. The file was modified as such:

```bash
- camera_auto_detect=1
+ camera_auto_detect=0
# [...]
[All]
+ dtoverlay=imx708,cam0
+ dtoverlay=imx708,cam1
+
+ # Enable external PCIe port on Pi5
+ dtparam=pciex1
```

This change in necessity to work with the ArduCams. Note that for an official Pi Camera Module 3, we didn't need to make this change. The camera was detected automatically and worked out of the box.

We also modify the boot order in the bootloader to ensure that we try to boot from an NVMe SSD in the PCIe interface before trying to boot from the SD card. This is done by modifying the `boot_order` parameter in the bootloader configuration.

```bash
## From https://www.jeffgeerling.com/blog/2023/nvme-ssd-boot-raspberry-pi-5
# Edit the EEPROM on the Raspberry Pi 5.
sudo rpi-eeprom-config --edit

# Change the BOOT_ORDER line to the following:
BOOT_ORDER=0xf416

# Add the following line if using a non-HAT+ adapter:
PCIE_PROBE=1

# Press Ctrl-O, then enter, to write the change to the file.
# Press Ctrl-X to exit nano (the editor).
```

Once these changes are made, it is recommended to reboot the Raspberry Pi to ensure that the changes take effect.

### UART Configuration

To enable UART on the Raspberry Pi for communicating with the Pi Pico, you need to modify the `/boot/firmware/config.txt` file, adding the following line:

```bash
# Enable UART
enable_uart=1
```

We then need to ensure that the UART is not being used by the console. We can do this through the `raspi-config` tool:

```bash
sudo raspi-config
```

Navigate to `Interface Options` -> `Serial Port` and disable the login shell over serial and enable the serial port hardware [source](www.raspberrypi.com/documentation/computers/configuration.html#disabling-the-linux-serial-console).

We prefer to communicate over the primary UART which defaults to GPIO 14 (TX) and GPIO 15 (RX).

### U2D2 udev Rule

Testing revealed that the U2D2 would often change its device path even while it was being used. To mitigate this, a udev rule was implemented to symlink the U2D2 device to a consistent path. See `/etc/udev/rules.d/98-dynamixel.rules`.

## Docker

The Docker container is built using the Dockerfile located in the `docker_config` directory. The container is based on the official ROS 2 Jazzy image and includes packages for using Pi Cameras and WiringPi GPIO tools. The container is built using the following command:

```bash
docker build -t vine_robot .
```
