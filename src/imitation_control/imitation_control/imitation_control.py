import rclpy
import json
from rclpy.node import Node
import socket
from std_msgs.msg import String
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import JointState
import numpy as np
import math
import os
import cv2
import glob

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('object_detection')
        self.publisher_ = self.create_publisher(JointState, '/joint_states', 10)
        timer_period = 0.04  # seconds
        self.timer1 = self.create_timer(timer_period, self.timer_callback)
        self.timer2 = self.create_timer(timer_period, self.timer_callback2)
        self.i = 0
        self.j = 0
        input_path = '/home/mihawk/output2/'
        self.npz_files = glob.glob(os.path.join(input_path, '*.npz'))
        self.image_files = glob.glob(os.path.join(input_path, '*.png'))
        self.npz_files.sort()
        self.image_files.sort()
        self.length = len(self.npz_files)
    def timer_callback2(self):
        if self.j >= self.length:
            return
        frame = cv2.imread(self.image_files[self.j])
        frame = cv2.resize(frame, (640, 480))
        if frame is not None:
            cv2.imshow('frame', frame)
            cv2.waitKey(1)
        else:
            self.get_logger().error(f"Failed to read image from {self.image_files[self.i]}")
        self.j += 1

        
    def timer_callback(self):
        if self.i >= self.length:
            return
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = ''
        msg.name = [
            'base_to_L1', 'L1_to_L2', 'L2_to_L3', 'L3_to_L4',
            'L4_to_L5_1_A', 'L4_to_L5_1_B', 'L4_to_L5_2_A',
            'L4_to_L5_2_B', 'L5_1_A_to_L5_3_A', 'L5_1_B_to_L5_3_B',
            'base_to_R1', 'R1_to_R2', 'R2_to_R3', 'R3_to_R4',
        ]
        msg.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        msg.velocity = []
        msg.effort = []
        self.publisher_.publish(msg)
        data = np.load(self.npz_files[self.i], allow_pickle=True)
        data =data['results'][()]
        thetas = data["smpl_thetas"][0].reshape(24, 3)
        r1 = R.from_rotvec(thetas[13]).as_matrix()
        r2 = R.from_rotvec(thetas[16]).as_matrix()
        
        r3 = R.from_rotvec(thetas[14]).as_matrix()
        r4 = R.from_rotvec(thetas[17]).as_matrix()
        r12_combined = r1 @ r2
        r12_combined_euler =  R.from_matrix(r12_combined).as_rotvec()
        
        r34_combined = r3 @ r4
        r34_combined_euler =  R.from_matrix(r34_combined).as_rotvec()
        
        msg.position[0] = r12_combined_euler[1]
        msg.position[1] = r12_combined_euler[2]-0.8
        msg.position[2] = -thetas[18][1]
        
        msg.position[10] = r34_combined_euler[1] + np.pi
        msg.position[11] = -r34_combined_euler[2]-0.8
        msg.position[12] = thetas[19][1]

        self.publisher_.publish(msg)
        self.i += 1
        # if self.i == 191:
        #     exit(0)


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