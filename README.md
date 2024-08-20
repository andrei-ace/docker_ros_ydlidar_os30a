```bash
cd ros
make build
```

```bash
xhost +local:docker
```

```bash
docker run -it --rm --privileged -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
    -e DISPLAY=$DISPLAY --net=host ros:noetic-ydlidar-os30a
```
```bash
source /ydlidar_os30a/bimu-rgbd-camera/devel/setup.bash
roslaunch bimu_rgbd_camera bimu_camera.launch
```

```bash
docker run -it --rm --privileged -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
    -e DISPLAY=$DISPLAY --net=host ros:noetic-eys3d-ros
```
```bash
source /eys3d_ros/devel/setup.bash
roslaunch dm_preview BMVM0S30A.launch
```