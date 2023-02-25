#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

from .oak1_test_yolov4 import test , labelMap
import depthai as dai
import time
import numpy as np



device_infos = dai.Device.getAllAvailableDevices()
if len(device_infos) == 0:
    print("No device found, exiting")
    exit()
else:
    print("Found", len(device_infos), "devices")
    for device_info in device_infos:
        print("=== Connected to " + device_info.getMxId())



class OAK1_ImagePublisher(Node):
  def __init__(self):
    super().__init__('depthai_OAK_1')

    topic_name= 'video_frames'
    self.publisher_ = self.create_publisher(Image, topic_name, 10)

    timer_period = 0.10  # seconds
    self.timer = self.create_timer(timer_period, self.timer_callback)
    self.br = CvBridge()


    self.syncNN = True
    self.pipeline = test()
    self.device = dai.Device()
    self.device.startPipeline(self.pipeline)

    self.qRgb = self.device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
    self.qDet = self.device.getOutputQueue(name="nn", maxSize=4, blocking=False)

    self.br = CvBridge()
    self.syncNN = True
    self.frame = None
    self.detections = []
    self.startTime = time.monotonic()
    self.counter = 0
    self.color = (255, 255, 255)

  def timer_callback(self):

    self.br = CvBridge()
    if self.syncNN:
      self.inRgb = self.qRgb.get()
      self.inDet = self.qDet.get()
    else:
      self.inRgb = self.qRgb.tryGet()
      self.inDet = self.qDet.tryGet()
    if self.inRgb is not None:
      self.frame = self.inRgb.getCvFrame()
      cv2.putText(self.frame, "NN fps: {:.2f}".format(self.counter / (time.monotonic() - self.startTime)),
                  (2, self.frame.shape[0] - 4), cv2.FONT_HERSHEY_TRIPLEX, 0.4, self.color)

      self.detections = self.inDet.detections
      self.counter += 1

      for detection in self.detections:
        self.bbox = (detection.xmin, detection.ymin, detection.xmax, detection.ymax)
        normVals = np.full(len(self.bbox), self.frame.shape[0])
        normVals[::2] = self.frame.shape[1]
        bbox = (np.clip(np.array(self.bbox), 0, 1) * normVals).astype(int)

        cv2.putText(self.frame, str(labelMap[detection.label]), (bbox[0] + 10, bbox[1] + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0),1)
        cv2.putText(self.frame, "{:.2f}".format(detection.confidence*100), (bbox[0] + 10, bbox[1] + 35), cv2.FONT_HERSHEY_SIMPLEX, 0.4, self.color,1)
        cv2.rectangle(self.frame, (bbox[0], bbox[1]), (bbox[2], bbox[3]), (0,0,225), 1)

      self.publisher_.publish(self.br.cv2_to_imgmsg(self.frame))
      self.get_logger().info('Publishing video frame')

def main(args=None):
  rclpy.init(args=args)
  oak1_image_publisher = OAK1_ImagePublisher()
  rclpy.spin(oak1_image_publisher)
  oak1_image_publisher.destroy_node()
  rclpy.shutdown()

if __name__ == '__main__':
  main()