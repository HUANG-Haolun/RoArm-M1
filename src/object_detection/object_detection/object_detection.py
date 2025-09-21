import rclpy
import json
from rclpy.node import Node
import socket
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation 
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import JointState
from time import time
import numpy as np
import math

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('object_detection')
        self.publisher_ = self.create_publisher(PoseStamped, '/object_pose', 10)
        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        host = '127.0.0.1' 
        port = 8888  
        self.s.bind((host, port))
        self.s.listen(1)
        self.get_logger().info('Waiting for a connection...')
        self.conn, addr = self.s.accept()

    def timer_callback(self):
        msg = PoseStamped()
        # msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera'
        
        # msg.header.seq = self.i
        # msg.data = 'Hello World: %d' % self.i
        # self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1
        try:

        
            self.get_logger().info("Received array:")
            data = self.conn.recv(1024)
            
            if not data:
                print("No data received")
            array_data = json.loads(data.decode('utf-8'))
            array_data = np.array(array_data)
            msg.header.stamp.sec = int(array_data[3][3])
            msg.pose.position.x = array_data[0][3]
            msg.pose.position.y = array_data[1][3]
            msg.pose.position.z = array_data[2][3]
            # msg.pose.position.z = 0.0
            rotation = array_data[:3,:3]
            r = R.from_matrix(rotation)

            quaternion = r.as_quat() 
            msg.pose.orientation.x = quaternion[0]
            msg.pose.orientation.y = quaternion[1]
            msg.pose.orientation.z = quaternion[2]
            msg.pose.orientation.w = quaternion[3]
            self.publisher_.publish(msg)

        except Exception as e:
            print("Error aaa:::", e)
            try:
                self.s.settimeout(1)
                self.conn, addr = self.s.accept()
            except socket.error as e:
                print(f"Socket accept error: {e}")

        


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()