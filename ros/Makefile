all: help

help:
	@echo ""
	@echo "-- Help Menu"
	@echo ""
	@echo "   1. make build            - build all images"
	@echo "   3. make clean            - remove all images"
	@echo ""

build:
	@docker build --tag=ros:noetic-ros-core-focal     																			ros-core/.
	@docker build --tag=ros:noetic-ros-base-focal     																			ros-base/.
	@docker build --tag=ros:noetic-robot-focal        																			robot/.
	@docker build --tag=ros:noetic-desktop-focal      																			desktop/.
	#@docker build --tag=ros:noetic-ydlidar-os30a 							--build-context ydlidar-os30a=../ydlidar_os30a   	ydlidar-os30a/.
	#@docker build --tag=ros:noetic-eys3d-ros     							--build-context eys3d-ros=../eys3d_ros				eys3d-ros/.
	@docker build --tag=andreiciobanu1984/robots:robot-dog-3d-depth-camera \
		--build-context eys3d-ros=../eys3d_ros \
		--build-context yolo-detector-ros=../yolo_detector_ros \
		--build-context detection-msgs=../detection_msgs \
		--build-context robot-dog=../robot_dog \
		robot-dog/.

clean:
	@docker rmi -f ros:noetic-ros-core-focal
	@docker rmi -f ros:noetic-ros-base-focal
	@docker rmi -f ros:noetic-robot-focal
	@docker rmi -f ros:noetic-desktop-focal
	#@docker rmi -f ros:noetic-ydlidar-os30a
	#@docker rmi -f ros:noetic-eys3d-ros
	@docker rmi -f andreiciobanu1984/robots:robot-dog-3d-depth-camera