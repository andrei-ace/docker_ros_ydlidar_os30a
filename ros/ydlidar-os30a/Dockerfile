FROM ros:noetic-desktop-focal
# install missing devel
RUN apt-get update && apt-get install -y --no-install-recommends \
    libudev-dev \
    && rm -rf /var/lib/apt/lists/*

COPY --from=ydlidar-os30a . /ydlidar_os30a

RUN /bin/bash -c '. /opt/ros/noetic/setup.bash; cd /ydlidar_os30a/bimu-rgbd-camera; catkin_make'