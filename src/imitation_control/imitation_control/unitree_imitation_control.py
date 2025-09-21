import rclpy
import json
from rclpy.node import Node
import socket
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation 
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import JointState
import numpy as np
import math
import os
import cv2
import glob
from threading import Lock
class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('object_detection')
        self.publisher_ = self.create_publisher(JointState, '/joint_states', 10)
        # timer_period = 0.1
        timer_period = 0.04
        self.timer1 = self.create_timer(timer_period, self.timer_callback)
        self.timer2 = self.create_timer(timer_period, self.timer_callback2)
        self.i = 0
        self.j = 0
        input_path = '/home/mihawk/output1/'
        self.npz_files = glob.glob(os.path.join(input_path, '*.npz'))
        self.image_files = glob.glob(os.path.join(input_path, '*.png'))
        self.npz_files.sort()
        self.image_files.sort()
        self.length = len(self.npz_files)
        self.lock = Lock() 
        self.pose = [0] * 29
    def timer_callback2(self):
        if self.j >= self.length -1:
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
        if self.i >= self.length -1:
            return
        if not self.lock.acquire(blocking=True):
            self.get_logger().info("Lock is busy, skipping this iteration.")
            return
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = ''
        msg.name = [
            'left_hip_pitch_joint', 'left_hip_roll_joint', 'left_hip_yaw_joint',
            'left_knee_joint', 'left_ankle_pitch_joint', 'left_ankle_roll_joint',
            'right_hip_pitch_joint', 'right_hip_roll_joint', 'right_hip_yaw_joint',#6 7 8
            'right_knee_joint', 'right_ankle_pitch_joint', 'right_ankle_roll_joint',
            'waist_yaw_joint', 'waist_roll_joint', 'waist_pitch_joint',
            'left_shoulder_pitch_joint', 'left_shoulder_roll_joint', 'left_shoulder_yaw_joint',# 15 16 17
            'left_elbow_joint', 'left_wrist_roll_joint', 'left_wrist_pitch_joint', 'left_wrist_yaw_joint', # 18 19 20 21
            'right_shoulder_pitch_joint', 'right_shoulder_roll_joint', 'right_shoulder_yaw_joint', # 22 23 24
            'right_elbow_joint', 'right_wrist_roll_joint', 'right_wrist_pitch_joint', 'right_wrist_yaw_joint' # 25 26 27 28
        ]
        msg.position = [0.0] * 29 
        msg.velocity = []
        msg.effort = []
        self.publisher_.publish(msg)
        data = np.load(self.npz_files[self.i], allow_pickle=True)
        data =data['results'][()]
        thetas = data["smpl_thetas"][0].reshape(24, 3)
        print(self.i)
        r1 = R.from_rotvec(thetas[13]).as_matrix()
        r2 = R.from_rotvec(thetas[16]).as_matrix()
        
        r3 = R.from_rotvec(thetas[14]).as_matrix()
        r4 = R.from_rotvec(thetas[17]).as_matrix()
        
        #CAUTION: the coordinate in smpl is camera frame, the robot is in world frame
        # x  ,y , z
        r12_combined = r1 @ r2
        # print(f"r12_combined_rotvec: {R.from_matrix(r12_combined).as_rotvec()}")
        # r12_combined_euler[2] -= np.pi/2
        r12_combined_euler = R.from_matrix(r12_combined).as_euler('xzx')
        # print(f"r12_combined_rotvec: {R.from_matrix(r12_combined).as_rotvec()}")
        
        r34_combined = r3 @ r4
        r34_combined_euler =  R.from_matrix(r34_combined).as_euler('xzx')
        # r_combined_euler: [-1.41144546 -0.18481408 -0.40975018]

        # print(f"r_combined_euler: {r12_combined_euler}")
        # print(f"r1_combined_euler: {r34_combined_euler}")
        left_hip = R.from_rotvec(thetas[1]).as_euler('yzx')
        right_hip = R.from_rotvec(thetas[2]).as_euler('yzx')
        
        r5 = R.from_rotvec(thetas[3]).as_matrix()
        r6 = R.from_rotvec(thetas[6]).as_matrix()
        r7 = R.from_rotvec(thetas[9]).as_matrix()

        r567_combined = r5 @ r6 @ r7
        r567_combined_euler = R.from_matrix(r567_combined).as_euler('xzy')
        msg.position[12] = r567_combined_euler[2]
        msg.position[13] = r567_combined_euler[1]
        msg.position[14] = r567_combined_euler[0]

        msg.position[0] = left_hip[2]
        msg.position[1] = left_hip[1]
        msg.position[2] = -left_hip[0]
        msg.position[3] = thetas[4][0]
        
        msg.position[6] = right_hip[2]
        msg.position[7] = right_hip[1]
        msg.position[8] = right_hip[0]
        msg.position[9] = thetas[5][0]
        
        
        msg.position[15] = r12_combined_euler[2]  
        msg.position[16] = r12_combined_euler[1]
        msg.position[17] = -r12_combined_euler[0]
        msg.position[18] = thetas[18][1]

        msg.position[22] = r34_combined_euler[2]
        msg.position[23] = r34_combined_euler[1]
        msg.position[24] = r34_combined_euler[0]
        msg.position[25] = -thetas[19][1]
        
        
        self.publisher_.publish(msg)
        self.i += 1
        self.lock.release()



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