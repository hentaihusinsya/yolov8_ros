import cv2 
import torch
import random


import rclpy
from rclpy.node import Node 
from cv_bridge import CvBridge

from ultralytics import YOLO

from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D
from vision_msgs.msg import BoundingBox2D
#from happymimi2_recognition_msgs import RecognitionList

from geometry_msgs.msg import Pose2D

from ultralytics import YOLO

class RecognitionTools(Node):

    def __init__(self):
        
        self.bridge = CvBridge()
        self.model = YOLO("yolov8n.pt")
        self.model.fuse()
        self.publisher_ = self.create_publisher(
            BoundingBox2D,
            'boundingbox',
            10
        )
        #self.srv = self.create_service(
            #RecognitionList,
            #'List',
            #self.hogehoge #決まってない

        #)
        self.subscriptions = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.ListObject, 
            10
        )
        self.subscriptions

    def process_image(self, msg):
        print("debug")
        orig = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        return orig
        

    def ListObject(self, image_msg):

        image_data = self.process_image(image_msg)

        results = self.model(image_data)

        frame = results[0].plot()
        boxes = results[0].boxes()

        x1 = boxes[0].xyxy[0][3].item()
        x2 = boxes[0].xyxy[0][2].item()
        y1 = boxes[0].xyxy[0][1].item()
        y2 = boxes[0].xyxy[0][0].item()
        
        w = int(round(x2 -x1))
        h = int(round(y2 - y1))
        cx = int(round(x1 + w / 2))
        cy = int(round(y1 + h / 2))

        bounding_boxes = BoundingBox2D()

        bounding_boxes.size_x = w
        bounding_boxes.size_y = h

        bounding_boxes.center = Pose2D()
        bounding_boxes.center.x = cx
        bounding_boxes.center.y = cy

        self.publisher_.publish(bounding_boxes)

def main(args=None):

    print("debug")
    rclpy.init(args=args)

    pub = RecognitionTools()

    rclpy.spin(pub)

    pub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


