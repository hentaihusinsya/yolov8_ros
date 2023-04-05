import rclpy
from rclpy.node import Node

from std_msgs.msg import String

from vision_msgs.msg import BoundingBox2D

from rclpy.qos import qos_profile_sensor_data

class yolov8_test(Node):

    def __init__(self):
        super().__init__('test')
        self.subscription = self.create_subscription(
            BoundingBox2D,
            '/boundingbox',
            self.callback,
            qos_profile_sensor_data
        )
        self.subscription

    def callback(self,msg):
        print("size_x" + str(msg.size_x))
        print("size_y" + str(msg.size_y))
        print("center_position_x" + str(msg.center.position.x))
        print("center_posiotion_y" + str(msg.center.position.y))

def main(args=None):
    rclpy.init(args=args)

    subsc = yolov8_test()

    rclpy.spin(subsc)

    subsc.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()