#! /usr/bin/env python3

import rclpy
from rclpy.node import Node 
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import mediapipe as mp
import time



class FaceMeshDet():
  def __init__(self,maxHands=2, detectionCon=0.5, trackCon=0.5):
    self.maxHands = maxHands
    self.detectionCon = detectionCon
    self.trackCon= trackCon

    self.mpFaceMesh = mp.solutions.face_mesh
    self.mp_face_mesh = self.mpFaceMesh.FaceMesh(max_num_faces=1, refine_landmarks=True, min_detection_confidence=0.5, min_tracking_confidence=0.5)
    self.mpDraw = mp.solutions.drawing_utils
    self.mp_drawing = mp.solutions.drawing_utils
    self.mp_drawing_styles = mp.solutions.drawing_styles

  def findLandmarks(self,img,draw=True):
    imgRGB = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    self.results = self.mp_face_mesh.process(imgRGB)
    
    if self.results.multi_face_landmarks:
      for face_landmarks in self.results.multi_face_landmarks:
        self.mp_drawing.draw_landmarks(
            image=img,
            landmark_list=face_landmarks,
            connections=self.mpFaceMesh.FACEMESH_TESSELATION,
            landmark_drawing_spec=None,
            connection_drawing_spec=self.mp_drawing_styles
            .get_default_face_mesh_tesselation_style())
        self.mp_drawing.draw_landmarks(
            image=img,
            landmark_list=face_landmarks,
            connections=self.mpFaceMesh.FACEMESH_CONTOURS,
            landmark_drawing_spec=None,
            connection_drawing_spec=self.mp_drawing_styles
            .get_default_face_mesh_contours_style())
        self.mp_drawing.draw_landmarks(
            image=img,
            landmark_list=face_landmarks,
            connections=self.mpFaceMesh.FACEMESH_IRISES,
            landmark_drawing_spec=None,
            connection_drawing_spec=self.mp_drawing_styles
            .get_default_face_mesh_iris_connections_style())

    return img


class mediapipeNode(Node):
    def __init__(self):
        super().__init__('mediapipe_demo')

        topic_name= 'video_frames'
        self.get_logger().info('initializing mediapipe..')
        self.detector = FaceMeshDet(detectionCon=0.3, trackCon=0.3)

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

