FROM arm64v8/ros:jazzy
LABEL description="Extends base OSRF ROS Official ARM64v8 image to support raspberry pi camera module 3 and GPIO operation"

ENV DEBIAN_FRONTEND=noninteractive

SHELL ["/bin/bash", "-c"]
WORKDIR /picam

# Install stable system packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    git \
    wget \
    cmake \
    meson \
    doxygen \
    openssl \
    i2c-tools \
    v4l-utils \
    build-essential

# Install development libraries
RUN apt-get update && apt-get install -y --no-install-recommends \
    libboost-dev \
    libgnutls28-dev \
    libtiff-dev \
    pybind11-dev \
    libglib2.0-dev \
    libgstreamer-plugins-base1.0-dev \
    qtbase5-dev \
    libqt5core5a \
    libqt5gui5 \
    libqt5widgets5 \
    gcc-arm-none-eabi \
    libnewlib-arm-none-eabi \
    libstdc++-arm-none-eabi-newlib

# Install Python packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3 \
    python3-pip \
    python3-jinja2 \
    python3-yaml \
    python3-ply \
    python3-smbus \
    python3-spidev \
    python3-rpi.gpio

# Install ROS packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-desktop=0.11.0-1* \
    ros-jazzy-usb-cam

# Clone and build Raspberry Pi's Libcamera fork
ARG LIBCAMERA_COMMIT_HASH="d83ff0a4ae4503bc56b7ed48cd142c3dd423ad3b"
WORKDIR /tmp
ADD https://github.com/raspberrypi/libcamera.git#"${LIBCAMERA_COMMIT_HASH}" /tmp/libcamera
RUN cd libcamera \
    && meson setup build --buildtype=release \
      -Dpipelines=rpi/vc4,rpi/pisp \
      -Dipas=rpi/vc4,rpi/pisp \
      -Dv4l2=true \
      -Dgstreamer=enabled \
      -Dtest=false \
      -Dlc-compliance=disabled \
      -Dcam=disabled \
      -Dqcam=disabled \
      -Ddocumentation=disabled \
      -Dpycamera=enabled \
    && ninja -C build install \
    && cd .. \
    && rm -rf /tmp/libcamera/

# Clone and build our fork of the camera_ros package
ADD "https://api.github.com/repos/MECH5080-Project-164/camera_ros/commits?per_page=1" latest_commit
RUN apt-get update && mkdir -p /picam/src \
    && cd /picam/src \
    && git clone https://github.com/MECH5080-Project-164/camera_ros.git \
    && source /opt/ros/$ROS_DISTRO/setup.bash \
    && cd /picam \
    && rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO --skip-keys=libcamera \
    && colcon build --event-handlers=console_direct+

# Install and build micro-ROS
WORKDIR /microros_ws
ADD https://github.com/micro-ROS/micro_ros_setup.git#"${ROS_DISTRO}" /microros_ws/src/micro_ros_setup
RUN source /opt/ros/$ROS_DISTRO/setup.bash \
    && apt-get update \
    && rosdep update \
    && rosdep install --from-paths src --ignore-src -y \
    && colcon build \
    && source install/setup.bash \
    && ros2 run micro_ros_setup create_agent_ws.sh \
    && ros2 run micro_ros_setup build_agent.sh

# Install and build Pi Pico SDK and micro-ROS for Raspberry Pi Pico
ENV PICO_TOOLCHAIN_PATH ...
ENV PICO_SDK_PATH /pico_ws/src/pico-sdk

ADD https://github.com/MECH5080-Project-164/micro_ros_raspberrypi_pico.git#jazzy /pico_ws/src/micro_ros_raspberrypi_pico
ADD https://github.com/raspberrypi/pico-sdk.git /pico_ws/src/pico-sdk
RUN cd /pico_ws/src \
    && git submodules update --init --recursive \
    && echo "export PICO_TOOLCHAIN_PATH=..." >> ~/.bashrc \
    && echo "export PICO_SDK_PATH=/pico_ws/src/pico-sdk" >> ~/.bashrc \
    && cd micro_ros_raspberrypi_pico \
    && mkdir build \
    && cd build \
    && cmake .. \
    && make

# Install precompiled WiringPi
WORKDIR /wiringpi
RUN wget -q https://github.com/WiringPi/WiringPi/releases/download/3.14/wiringpi_3.14_arm64.deb \
    && dpkg -i wiringpi_3.14_arm64.deb \
    && rm wiringpi_3.14_arm64.deb \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# Configure the entry point
COPY ./docker_entrypoint.sh /
COPY ./ros_env_setup.sh /
RUN chmod +x /docker_entrypoint.sh /ros_env_setup.sh \
    && echo "source /ros_env_setup.sh" >> /root/.bashrc

ENTRYPOINT ["/docker_entrypoint.sh"]

WORKDIR /home/workspace
CMD ["bash"]
