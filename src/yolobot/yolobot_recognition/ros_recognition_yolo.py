#!/usr/bin/env python3
import os, sys
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError


import cv2
import numpy as np
import yolov5
import math
import time


bridge = CvBridge()

class Camera_subscriber(Node):

    def __init__(self):
        super().__init__('camera_subscriber')


        # Load the YOLOv5 model
        self.model_path = './yolov5s.pt'

        self.device = "cpu"  # for cpu
        # self.device = 0  #for gpu
        self.yolov5 = yolov5.YOLOv5(self.model_path,self.device,load_on_init=True)

        # Initialize
        # set_logging()

        self.subscription = self.create_subscription(
        Image,
        'rgb_cam/image_raw',
        self.camera_callback,
        10)
        self.subscription  # prevent unused variable warning

    def camera_callback(self, data):
        # print(data)
        t0 = time.time()
        frame = bridge.imgmsg_to_cv2(data, "bgr8")

        # Perform object detection on the frame
        results = self.yolov5.predict(frame, size = 640, augment=False)
        detections = results.pred[0]
        COLORS = np.random.uniform(0,255,size=(80,3))
        names = results.names
        labels_map = names

        # Check whether the bounding box centroids are inside the ROI
        for detection in detections:
            # print(detection)    
            class_id  = detection[5]
            class_id  = math.floor(class_id)
            label     = labels_map[class_id] if labels_map and len(labels_map) >= class_id else str(class_id)
            color     = COLORS[class_id]

            (tw, th), _ = cv2.getTextSize(str(label), cv2.FONT_HERSHEY_SIMPLEX, 0.6, 1)
            cv2.rectangle(frame, (int(detection[0]), int(detection[1])), (int(detection[2]), int(detection[3])), (0,0,225), 2)
            cv2.rectangle(frame, (int(detection[0]), int(detection[1]) - 20), (int(detection[0]) + tw, int(detection[1])), (20,20,20), -1)
            cv2.putText(frame, str(label), (int(detection[0]), int(detection[1]-5)), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255,255,255), 1, cv2.LINE_AA)


        # Display the frame
        cv2.imshow("IMAGE", frame)
        cv2.waitKey(4)  

def main():
    rclpy.init(args=None)
    camera_subscriber = Camera_subscriber()
    rclpy.spin(camera_subscriber)
    rclpy.shutdown()



if __name__ == '__main__':
    main()