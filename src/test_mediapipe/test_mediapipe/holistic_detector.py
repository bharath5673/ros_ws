#! /usr/bin/env python3

import rclpy
from rclpy.node import Node 
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import mediapipe as mp
import time



class Holistic():
    def __init__(self,mode=False,maxHands=2, detectionCon=0.5, trackCon=0.5):
        self.mode = mode
        self.maxHands = maxHands
        self.detectionCon = detectionCon
        self.trackCon= trackCon

        self.mpHolistic = mp.solutions.holistic
        self.holistic = self.mpHolistic.Holistic(self.mode, self.maxHands, self.trackCon)
        self.mpDraw = mp.solutions.drawing_utils

    def findLandmarks(self,img,draw=False):
        imgRGB = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        self.results = self.holistic.process(imgRGB)
        img = cv2.cvtColor(imgRGB, cv2.COLOR_RGB2BGR)
        # print(self.results.right_hand_landmarks)
        # print(self.results.left_hand_landmarks)

        if draw:
            # # 1. Draw face landmarks
            self.mpDraw.draw_landmarks(img, self.results.face_landmarks, self.mpHolistic.FACEMESH_CONTOURS,
                                     self.mpDraw.DrawingSpec(color=(80,110,10), thickness=1, circle_radius=1),
                                     self.mpDraw.DrawingSpec(color=(80,256,121), thickness=1, circle_radius=1))

            # 2. Right hand
            self.mpDraw.draw_landmarks(img, self.results.right_hand_landmarks, self.mpHolistic.HAND_CONNECTIONS,
                                     self.mpDraw.DrawingSpec(color=(80,22,10), thickness=1, circle_radius=2),
                                     self.mpDraw.DrawingSpec(color=(80,44,121), thickness=1, circle_radius=1))

            # 3. Left Hand
            self.mpDraw.draw_landmarks(img, self.results.left_hand_landmarks, self.mpHolistic.HAND_CONNECTIONS,
                                     self.mpDraw.DrawingSpec(color=(80,22,10), thickness=1, circle_radius=2),
                                     self.mpDraw.DrawingSpec(color=(80,44,121), thickness=1, circle_radius=1))

            # 4. Pose Detections
            self.mpDraw.draw_landmarks(img, self.results.pose_landmarks, self.mpHolistic.POSE_CONNECTIONS,
                                     self.mpDraw.DrawingSpec(color=(245,117,66), thickness=1, circle_radius=2),
                                     self.mpDraw.DrawingSpec(color=(245,66,230), thickness=1, circle_radius=1))
        return img

 


    
class mediapipeNode(Node):
    def __init__(self):
        super().__init__('mediapipe_demo')

        topic_name= 'video_frames'
        self.get_logger().info('initializing mediapipe..')
        self.detector = Holistic(detectionCon=0.3, trackCon=0.3)

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
        processed_img = self.detector.findLandmarks(current_frame, draw=True)
        self.get_logger().info('mediapipe processed img')

        cv2.imshow("demo", processed_img)   
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = mediapipeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

  
if __name__ == '__main__':
  main()