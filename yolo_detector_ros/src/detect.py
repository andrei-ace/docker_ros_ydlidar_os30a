#!/usr/bin/env python3

import ros_numpy
import rospy

import ultralytics
from ultralytics import YOLO

from rostopic import get_topic_type
from sensor_msgs.msg import Image, CompressedImage
from detection_msgs.msg import BoundingBox, BoundingBoxes


class YoloDetector:
    def __init__(self):
        self.conf_thres = float(rospy.get_param("~confidence_threshold"))
        self.iou_thres = float(rospy.get_param("~iou_threshold"))
        self.agnostic_nms = bool(rospy.get_param("~agnostic_nms"))
        self.max_det = int(rospy.get_param("~maximum_detections"))
        # Initialize weights 
        weights = rospy.get_param("~weights")
        self.model = YOLO(weights, task="detect")        
        ultralytics.checks()

        # Initialize subscriber to Image/CompressedImage topic
        input_image_type, input_image_topic, _ = get_topic_type(rospy.get_param("~input_image_topic"), blocking = True)
        self.compressed_input = input_image_type == "sensor_msgs/CompressedImage"

        if self.compressed_input:
            self.image_sub = rospy.Subscriber(
                input_image_topic, CompressedImage, self.image_callback, queue_size=1
            )
        else:
            self.image_sub = rospy.Subscriber(
                input_image_topic, Image, self.image_callback, queue_size=1
            )

        # Initialize prediction publisher
        self.pred_pub = rospy.Publisher(
            rospy.get_param("~output_topic"), BoundingBoxes, queue_size=10
        )
        # Initialize image publisher
        self.publish_image = rospy.get_param("~publish_image")
        if self.publish_image:
            self.image_pub = rospy.Publisher(
                rospy.get_param("~output_image_topic"), Image, queue_size=10
            )
        self.verbose = rospy.get_param("~verbose")

        self.detection_rate = rospy.get_param("~detection_rate", 0.0)  # in Hz, 0 means process every frame
        self.last_image = None

        if self.detection_rate > 0:
            rospy.Timer(rospy.Duration(1.0 / self.detection_rate), self.timer_callback)
        else:
            rospy.Timer(rospy.Duration(0.01), self.timer_callback)  # 100 Hz timer for immediate processing

    def image_callback(self, data):
        """Store the most recent image."""
        self.last_image = data

    def timer_callback(self, event):
        """Process the most recent image at the specified rate."""
        if self.last_image is not None:
            self.process_image(self.last_image)
            self.last_image = None  # Reset to avoid processing the same image twice

    def process_image(self, data):
        """Process image and publish detected classes."""
        try:
            array = ros_numpy.numpify(data)
            
            pred = self.model(array, 
                              conf=self.conf_thres,
                              iou=self.iou_thres,                         
                              max_det=self.max_det, 
                              agnostic_nms=self.agnostic_nms,
                              verbose=self.verbose
                              )    
            
            bounding_boxes = BoundingBoxes()
            bounding_boxes.header = data.header
            bounding_boxes.image_header = data.header
        
            for result in pred:
                for box in result.boxes:
                    bounding_box = BoundingBox()
                    c = int(box.cls)
                    bounding_box.Class = result.names[c]
                    bounding_box.probability = float(box.conf.item())
                    bounding_box.xmin = int(box.xyxy[0,0].item())
                    bounding_box.ymin = int(box.xyxy[0,1].item())
                    bounding_box.xmax = int(box.xyxy[0,2].item())
                    bounding_box.ymax = int(box.xyxy[0,3].item())

                    bounding_boxes.bounding_boxes.append(bounding_box)

                if self.publish_image:
                    im_bgr = result.plot()
                    img_msg = ros_numpy.msgify(Image, im_bgr, encoding=data.encoding)
                    img_msg.header = data.header
                    self.image_pub.publish(img_msg)

            self.pred_pub.publish(bounding_boxes)
            
            if self.verbose:
                rospy.loginfo(f"Published {len(bounding_boxes.bounding_boxes)} detections")
        
        except Exception as e:
            rospy.logerr(f"Error in YoloDetector process_image: {str(e)}")


if __name__ == "__main__":    
    rospy.init_node("yolo_detector", anonymous=True)
    detector = YoloDetector()
    rospy.spin()