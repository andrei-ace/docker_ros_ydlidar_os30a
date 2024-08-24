#!/usr/bin/env python3

import time

import ros_numpy
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String

import ultralytics
from ultralytics import YOLO


# Load the model
model = YOLO("yolov5nu.pt", task="detect")
ultralytics.checks()

rospy.init_node("ultralytics", disable_signals=True)
time.sleep(1)
classes_pub = rospy.Publisher("/ultralytics/detection/classes", String, queue_size=5)

def callback(data):
    """Callback function to process image and publish detected classes."""
    array = ros_numpy.numpify(data)
    if classes_pub.get_num_connections():
        det_result = model(array)
        classes = det_result[0].boxes.cls.cpu().numpy().astype(int)
        names = [det_result[0].names[i] for i in classes]
        classes_pub.publish(String(data=str(names)))


rospy.Subscriber("/dm_preview/BMVM0S30A1/left/image_color", Image, callback, queue_size=1)
rospy.spin()