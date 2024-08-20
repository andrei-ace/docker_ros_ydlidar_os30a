FROM ros:noetic-desktop-focal

# Install the dependency packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev libv4l-dev libgtk-3-dev libusb-dev \
    python-dev python-numpy libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev libdc1394-22-dev libjpeg9 \
    && rm -rf /var/lib/apt/lists/*

COPY --from=eys3d-ros . /eys3d_ros/src
RUN /bin/bash -c '. /opt/ros/noetic/setup.bash; cd /eys3d_ros; catkin_make'