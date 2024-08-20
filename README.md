## Build the images
```bash
cd ros
make build
```

## Allow X Server to accept connections from docker
```bash
xhost +local:docker
```

## Run the docker image with the SDK proviede by YDLIDAR https://www.ydlidar.com/service_support/download.html?gid=23
```bash
docker run -it --rm --privileged -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
    -e DISPLAY=$DISPLAY --net=host ros:noetic-ydlidar-os30a
```
### Start the ros node
```bash
source /ydlidar_os30a/bimu-rgbd-camera/devel/setup.bash
roslaunch bimu_rgbd_camera bimu_camera.launch
```


## Run the docker image for https://github.com/eYs3D/eys3d_ros/
```bash
docker run -it --rm --privileged -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
    -e DISPLAY=$DISPLAY --net=host ros:noetic-eys3d-ros
```
### Start the ros node
```bash
source /eys3d_ros/devel/setup.bash
roslaunch dm_preview BMVM0S30A.launch
```