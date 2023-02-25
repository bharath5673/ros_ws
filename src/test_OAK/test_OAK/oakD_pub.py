#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from .oakD_test_yolov4 import test , labelMap
import depthai as dai
import time


class OAKD_ImagePublisher(Node):
    def __init__(self):
        super().__init__('depthai_OAK_D')

        topic_name= 'video_frames'

        self.publisher_ = self.create_publisher(Image, topic_name , 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.br = CvBridge()
        self.syncNN = True

        self.get_logger().info('Initializing OAK-D camera capture node')

        self.syncNN = True
        self.pipeline = test()
        self.device = dai.Device()
        self.device.startPipeline(self.pipeline)

        # Output queues will be used to get the rgb frames and nn data from the outputs defined above
        self.previewQueue = self.device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
        self.detectionNNQueue = self.device.getOutputQueue(name="detections", maxSize=4, blocking=False)
        self.xoutBoundingBoxDepthMapping = self.device.getOutputQueue(name="boundingBoxDepthMapping", maxSize=4, blocking=False)
        self.depthQueue = self.device.getOutputQueue(name="depth", maxSize=4, blocking=False)

        self.frame = None
        self.detections = []

        self.startTime = time.monotonic()
        self.counter = 0
        self.fps = 0
        self.color = (255, 255, 255)


    def timer_callback(self):
        self.br = CvBridge()
        if self.syncNN:
            self.inPreview = self.previewQueue.get()
            self.inNN = self.detectionNNQueue.get()
            self.depth = self.depthQueue.get()
        else:
            self.inPreview = self.previewQueue.tryGet()
            self.inNN = self.detectionNNQueue.tryGet()
            self.depth = self.depthQueue.tryGet()

        self.counter+=1
        self.current_time = time.monotonic()
        if (self.current_time - self.startTime) > 1 :
            self.fps = self.counter / (self.current_time - self.startTime)
            self.counter = 0
            self.startTime = self.current_time

        if self.inPreview is not None:
            self.frame = self.inPreview.getCvFrame()
            self.depthFrame = self.depth.getFrame()
            self.objects = list()


            self.depthFrameColor = cv2.normalize(self.depthFrame, None, 255, 0, cv2.NORM_INF, cv2.CV_8UC1)
            self.depthFrameColor = cv2.equalizeHist(self.depthFrameColor)
            self.depthFrameColor = cv2.applyColorMap(self.depthFrameColor, cv2.COLORMAP_HOT)
            self.detections = self.inNN.detections


            self.h, self.w = self.frame.shape[1],self.frame.shape[0]
            self.width_cutoff = self.w // 2
            self.right = self.frame[:,:self.width_cutoff]
            self.left = self.frame[:,self.width_cutoff:]


            if len(self.detections) != 0:
                self.boundingBoxMapping = self.xoutBoundingBoxDepthMapping.get()
                self.roiDatas = self.boundingBoxMapping.getConfigData()

                for roiData in self.roiDatas:
                    roi = roiData.roi
                    roi = roi.denormalize(self.depthFrameColor.shape[1], self.depthFrameColor.shape[0])
                    topLeft = roi.topLeft()
                    bottomRight = roi.bottomRight()
                    xmin = int(topLeft.x)
                    ymin = int(topLeft.y)
                    xmax = int(bottomRight.x)
                    ymax = int(bottomRight.y)
                    cv2.rectangle(self.depthFrameColor, (xmin, ymin), (xmax, ymax), self.color, cv2.FONT_HERSHEY_SCRIPT_SIMPLEX)


            # If the frame is available, draw bounding boxes on it and show the frame
            self.img_h = self.frame.shape[0]
            self.img_w  = self.frame.shape[1]
            for detection in self.detections:
                # Denormalize bounding box
                x1 = int(detection.xmin * self.img_w)
                x2 = int(detection.xmax * self.img_w)
                y1 = int(detection.ymin * self.img_h)
                y2 = int(detection.ymax * self.img_h)
                try:
                    label = labelMap[detection.label]
                except:
                    label = detection.label

                cv2.putText(self.frame, str(label), (x1 + 10, y1 + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0),1)
                cv2.putText(self.frame, "{:.2f}".format(detection.confidence*100), (x1 + 10, y1 + 35), cv2.FONT_HERSHEY_SIMPLEX, 0.4, self.color,1)
                cv2.putText(self.frame, f"X: {int(detection.spatialCoordinates.x)/100} cm", (x1 + 10, y1 + 50), cv2.FONT_HERSHEY_SIMPLEX, 0.4, self.color,1)
                cv2.putText(self.frame, f"Y: {int(detection.spatialCoordinates.y)/100} cm", (x1 + 10, y1 + 65), cv2.FONT_HERSHEY_SIMPLEX, 0.4, self.color,1)
                cv2.putText(self.frame, f"Z: {int(detection.spatialCoordinates.z)/100} cm", (x1 + 10, y1 + 80), cv2.FONT_HERSHEY_SIMPLEX, 0.4, self.color,1)
                cv2.rectangle(self.frame, (x1, y1), (x2, y2), (0,0,225), 1)
            cv2.putText(self.frame, "NN fps: {:.2f}".format(self.fps), (2, self.frame.shape[0] - 4), cv2.FONT_HERSHEY_TRIPLEX, 0.4, self.color)

        self.publisher_.publish(self.br.cv2_to_imgmsg(self.frame))
        self.get_logger().info('Publishing video frame')


def main(args=None):
    rclpy.init(args=args)
    oakD_ImagePublisher = OAKD_ImagePublisher()
    rclpy.spin(oakD_ImagePublisher)
    oakD_ImagePublisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
  main()