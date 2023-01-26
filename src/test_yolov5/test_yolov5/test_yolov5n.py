#! /usr/bin/env python3

import rclpy
from rclpy.node import Node 
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import time
import torch
import numpy as np
import math

class YOLOV5():
    def __init__(self, detectionCon=0.5, trackCon=0.5):
        self.detectionCon = detectionCon
        self.trackCon= trackCon
        # self.device = device
        self.model = torch.hub.load('ultralytics/yolov5', 'yolov5n')

    def process(self,img):
        frame = img.copy()
        self.results = self.model(frame)
        self.COLORS = np.random.uniform(0,255,size=(80,3))
        color_image = frame.copy()
        self.predictions = self.results.pred[0]
        self.names = self.results.names
        self.labels_map = self.names

        for detection in self.predictions.tolist():
            xmin    = detection[0]
            ymin    = detection[1]
            xmax    = detection[2]
            ymax    = detection[3]
            score   = detection[4]
            class_id= detection[5]

            # center_y = int((ymin + ymax) / 2)
            center_y = int((((ymin + ymax) / 2) + ymax) / 2)
            center_x = int((xmin + xmax) / 2)

            if class_id == 0 or class_id == 1 or class_id == 2 or class_id == 3:
                class_id  = math.floor(class_id)
                label     = self.labels_map[class_id] if self.labels_map and len(self.labels_map) >= class_id else str(class_id)

                cv2.circle(color_image, (center_x, center_y), 5, (255, 255, 255), -1)
                (tw, th), _ = cv2.getTextSize(str(label), cv2.FONT_HERSHEY_SIMPLEX, 0.6, 1)
                cv2.rectangle(color_image, (int(detection[0]), int(detection[1])), (int(detection[2]), int(detection[3])), (0,0,255), 2)
                cv2.rectangle(color_image, (int(detection[0]), int(detection[1]) - 20), (int(detection[0]) + tw, int(detection[1])), (20,20,20), -1)
                cv2.putText(color_image, str(label), (int(detection[0]), int(detection[1]-5)), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255,255,255), 1, cv2.LINE_AA)


        return color_image



    
class Yolov5Node(Node):
    def __init__(self):
        super().__init__('YOLO_v5_demo')

        topic_name= 'video_frames'
        self.get_logger().info('initializing yolov5..')
        self.detector = YOLOV5()

        self.publisher_ = self.create_publisher(Image, topic_name , 1)
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.cap = cv2.VideoCapture(0)
        self.br = CvBridge()

        self.subscription = self.create_subscription(Image, topic_name, self.img_callback, 1)
        self.subscription 
        self.br = CvBridge()


    def timer_callback(self):
        ret, frame = self.cap.read()     
        if ret == True:
            self.publisher_.publish(self.br.cv2_to_imgmsg(frame))
        self.get_logger().info('Publishing frame for process')


    def img_callback(self, data):
        current_frame = self.br.imgmsg_to_cv2(data)

        ##dectect
        processed_img = self.detector.process(current_frame)
        self.get_logger().info('yolov5 processed img')

        cv2.imshow("demo", processed_img)   
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = Yolov5Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

  
if __name__ == '__main__':
  main()