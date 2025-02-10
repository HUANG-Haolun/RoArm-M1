import rclpy
from rclpy.node import Node
import array

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped

import json
import serial

ser = serial.Serial("/dev/ttyUSB0",115200)
j_mode = 0
object_type = {"bottle","can","carton"}
stand_state = None

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('cv_ctrl')
        
        self.position = []
        
        self.subscription = self.create_subscription(
            PoseStamped,
            '/object_pose',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
    
        
    # def angleGet(self, radInput, multiInput):
    #     getAngle = int()

    def listener_callback(self, msg):

        
        print(msg)


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
