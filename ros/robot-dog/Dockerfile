FROM ros:noetic-desktop-focal

# Install the dependency packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev libv4l-dev libgtk-3-dev libusb-dev \
    python3-pip python-dev python-numpy libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev libdc1394-22-dev libjpeg9 \
    && rm -rf /var/lib/apt/lists/*

COPY --from=yolo-detector-ros      requirements.txt    /robot/requirements.txt
RUN /bin/bash -c 'pip3 install -r /robot/requirements.txt'


COPY --from=eys3d-ros               ./dm_preview        /robot/src/dm_preview
COPY --from=detection-msgs          .                   /robot/src/detection_msgs
COPY --from=yolo-detector-ros       .                   /robot/src/yolo_detector_ros
COPY --from=robot-dog               .                   /robot/src/robot_dog

RUN /bin/bash -c 'cd /robot/src/yolo_detector_ros/src; chmod +x detect.py'
RUN /bin/bash -c '. /opt/ros/noetic/setup.bash; cd /robot; catkin_make'
RUN /bin/bash -c 'yolo export model=yolov8n.pt format=ncnn'