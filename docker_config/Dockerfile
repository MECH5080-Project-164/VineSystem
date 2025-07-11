FROM arm64v8/ros:jazzy
LABEL description="Extends base OSRF ROS Official ARM64v8 image to support raspberry pi camera module 3 and GPIO operation"

ENV DEBIAN_FRONTEND=noninteractive

SHELL ["/bin/bash", "-c"]
WORKDIR /picam

RUN apt-get update && apt-get install -y --no-install-recommends \
    # Install ros2 desktop packages
    ros-jazzy-desktop=0.11.0-1* \
    qtbase5-dev \
    libqt5core5a \
    libqt5gui5 \
    libqt5widgets5 \
    # Install Dependencies for libcamera
    git \
    python3-pip \
    python3-jinja2 \
    libboost-dev \
    libgnutls28-dev \
    openssl \
    libtiff-dev \
    pybind11-dev \
    meson \
    cmake \
    python3-yaml \
    python3-ply \
    libglib2.0-dev \
    libgstreamer-plugins-base1.0-dev \
    # Install ROS Usb Cam package
    ros-jazzy-usb-cam \
    # Install dependencies for WiringPi
    wget \
    # Other dependencies
    v4l-utils

# Clone and build Raspberry Pi's Libcamera fork
ARG LIBCAMERA_COMMIT_HASH="d83ff0a4ae4503bc56b7ed48cd142c3dd423ad3b"
RUN git clone https://github.com/raspberrypi/libcamera.git \
    && cd libcamera \
    && git checkout "${LIBCAMERA_COMMIT_HASH}" \
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
    && rm -rf libcamera/.git

# Clone and build our fork of the camera_ros package
ADD "https://api.github.com/repos/MECH5080-Project-164/camera_ros/commits?per_page=1" latest_commit
RUN apt-get update && mkdir -p /picam/src \
    && cd /picam/src \
    && git clone https://github.com/MECH5080-Project-164/camera_ros.git \
    && source /opt/ros/$ROS_DISTRO/setup.bash \
    && cd /picam \
    && rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO --skip-keys=libcamera \
    && colcon build --event-handlers=console_direct+

# Configure the entry point
COPY ./docker_entrypoint.sh /picam/
ENTRYPOINT ["/picam/docker_entrypoint.sh"]

WORKDIR /wiringpi

# Install precompiled WiringPi
RUN wget -q https://github.com/WiringPi/WiringPi/releases/download/3.14/wiringpi_3.14_arm64.deb \
    && dpkg -i wiringpi_3.14_arm64.deb \
    && rm wiringpi_3.14_arm64.deb \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

USER root
WORKDIR /home/workspace

CMD ["bash", "-i", "-c", "source /opt/ros/$ROS_DISTRO/setup.bash && bash"]
