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
from happymimi2_recognition_msgs.srv import RecognitionList, MultipleLocalize, RecognitionCount.srv



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

        self.srv = self.create_service(RecognitionList)

        
        self.subscription

       
        #timer_period = 0.5
        #self.timer = self.create_timer(timer_period, self.ListObject)

    def process_image(self, data):
        print("debug1")
        orig = self.cv_bridge.imgmsg_to_cv2(data, "bgr8")
        return orig
    
    def Count_Object(self, )
        

    def List_Object(self, msg):

        print("debug2")
        image_data = self.process_image(msg)
        results = self.yolo(image_data, show=True, conf=0.3)

        frame = results[0].plot()
        boxes = results[0].boxes
        names = results[0].names

        print(names)
        #a = len(results[0])
        b = len(boxes)
        print(b)
        #print(a)
        #print(b)

        c = boxes[1].cls[0].item()
        #print(c)


        self.coordinate = [[] for i in range(b+1)]
        
        for i in range(b -1):

            ''''
            print(boxes[i].xyxy[0][0].item())
            print(boxes[i''].xyxy[0][1].item())
            print(boxes[i].xyxy[0][2].item())
            print(boxes[i].xyxy[0][3].item())
            '''

            x1 = boxes[i].xyxy[0][0].item()
            x2 = boxes[i].xyxy[0][1].item()
            y1 = boxes[i].xyxy[0][2].item()
            y2 = boxes[i].xyxy[0][3].item()
            frame_id = boxes[i].cls[0].item()


                    
            w = float(round(x2 -x1))
            h = float(round(y2 - y1))
            cx = float(round(x1 + w / 2))
            cy = float(round(y1 + h / 2))
            offset_cx = 640/2 - cx
            offset_cy = 480/2 - cy
            
            #print(w)
            #print(h)
            #print(cx)
            #print(cy)
            #print(frame_id)
            
            
            
            

            self.coordinate[0].append(w)
            self.coordinate[1].append(h)
            self.coordinate[2].append(cx)
            self.coordinate[3].append(cy)
            self.coordinate[4].append(640/2 - cx)
            self.coordinate[5].append(480/2 - cy)
            self.coordinate[6].append(frame_id)
            #print(self.coordinate)
            for i in range(b -1):
                bounding_box = BoundingBox2D()
                bounding_box.size_x = self.coordinate[0][i]
                bounding_box.size_y = self.coordinate[1][i]
                bounding_box.center.position.x = self.coordinate[4]
                bounding
            
            

        
        bounding_boxes = BoundingBox2DArray()
        bounding_box = BoundingBox2D()
        bounding_boxes.boxes.append(self.coordinate)
        #bounding_boxes.header.frame_id 

            


        
        print(self.coordinate)
            
                
       
        #print(self.coordinate[0])
        #print(self.coordinate[1])
        #print(self.coordinate[2])
        #print(self.coordinate[3])
        #print(self.coordinate[4])
        #print(self.coordinate[5])
        

        #bounding_boxes = BoundingBox2D()

        #bounding_boxes.size_x = w
        #bounding_boxes.size_y = h

        
        #bounding_boxes.center.position.x = offset_cx
        #bounding_boxes.center.position.y = offset_cy

        #self.publisher_.publish(bounding_boxes)

def main(args=None):

    print("debug3")
    rclpy.init(args=args)

    yolov8_test = RecognitionTools()

    rclpy.spin(yolov8_test)

    yolov8_test.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


