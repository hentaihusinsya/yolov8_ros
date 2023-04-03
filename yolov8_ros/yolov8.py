import cv2 
import torch
import random
import numpy as np


import rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.node import Node 
from cv_bridge import CvBridge

from ultralytics import YOLO

from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D
from vision_msgs.msg import BoundingBox2DArray, BoundingBox2D
#from happymimi2_recognition_msgs import RecognitionList

from geometry_msgs.msg import Pose2D

from ultralytics import YOLO

class RecognitionTools(Node):

    def __init__(self):
        super().__init__('yolov8_test')
        self.cv_bridge = CvBridge()
        self.yolo = YOLO("yolov8n.pt")
        self.yolo.fuse()
        self.publisher_ = self.create_publisher(
            BoundingBox2D,
            'boundingbox',
            10
        )
        
        self.subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.List_Object,
            qos_profile_sensor_data
        )
        self.subscription

        #timer_period = 0.5
        #self.timer = self.create_timer(timer_period, self.ListObject)

    def process_image(self, data):
        print("debug1")
        orig = self.cv_bridge.imgmsg_to_cv2(data, "bgr8")
        return orig
        

    def List_Object(self, msg):

        print("debug2")
        image_data = self.process_image(msg)
        results = self.yolo(image_data, show=True, conf=0.7)

        frame = results[0].plot()
        boxes = results[0].boxes
        names = results[0].names

        print(names)
        #a = len(results[0])
        b = len(boxes)
        #print(a)
        print(b)

        c = boxes[1].cls[0].item()
        print(c)
        x1 = boxes[0].xyxy[0][0].item()
        x2 = boxes[0].xyxy[0][1].item()
        y1 = boxes[0].xyxy[0][2].item()
        y2 = boxes[0].xyxy[0][3].item()

        coordinate = [[]] * 6

        for i in (b -1):

            x1 = boxes[i].xyxy[0][0].item()
            x2 = boxes[i].xyxy[0][1].item()
            y1 = boxes[i].xyxy[0][2].item()
            y2 = boxes[i].xyxy[0][3].item()
            

            for i in range(b):
                coordinate[0].append(float(round(x2 -x1)))
                coordinate[1].append(float(round(y2 -y1)))
                coordinate[2].append(int(round(x1 + w / 2)))
                coordinate[3].append(int(round(y1 + h / 2)))
                coordinate[4].append(640/2 - 
                coordinate[5].append(480/2 - cy)
                
        
            #w = float(round(x2 -x1))
            #h = float(round(y2 - y1))
            #cx = int(round(x1 + w / 2))
            #cy = int(round(y1 + h / 2))
            #offset_cx = 640/2 - cx
            #offset_cy = 480/2 - cy


        bounding_boxes = BoundingBox2D()

        bounding_boxes.size_x = w
        bounding_boxes.size_y = h

        
        bounding_boxes.center.position.x = offset_cx
        bounding_boxes.center.position.y = offset_cy

        self.publisher_.publish(bounding_boxes)

def main(args=None):

    print("debug3")
    rclpy.init(args=args)

    yolov8_test = RecognitionTools()

    rclpy.spin(yolov8_test)

    yolov8_test.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


