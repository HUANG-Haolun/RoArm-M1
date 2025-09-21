import rclpy
from rclpy.node import Node
import array

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from std_msgs.msg import Int16
from geometry_msgs.msg import PoseStamped
import tf2_ros
import json
import serial
import tf2_geometry_msgs
import time

from piper_msgs.msg import PosCmd
from rclpy.node import Node
import socket
from scipy.spatial.transform import Rotation 
from scipy.spatial.transform import Rotation as R
import numpy as np
import math
from tf2_ros import TransformException
# ser = serial.Serial("/dev/ttyUSB1",115200) 
object_type = {"bottle","can","carton"}
stand_state = None
target_x = 180.5104065
target_y = -13.75
target_z = 186.5822754
target_angle = 235
grab_state = False
object_type = -1
class MinimalSubscriber(Node):
    def setCmd(self, x=0.0, y=0.0, z=0.0, roll=0.0, pitch=0.0, yaw=0.0, gripper=0.0):
        pose_cmd = PosCmd()
        pose_cmd.x = float(x)
        pose_cmd.y = float(y)
        pose_cmd.z = float(z)
        pose_cmd.roll = float(roll)
        pose_cmd.pitch = float(pitch)
        pose_cmd.yaw = float(yaw)
        pose_cmd.gripper = gripper
        self.cmd_pub.publish(pose_cmd)
    
    def __init__(self):
        super().__init__('cv_ctrl')
        
        self.position = []
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)  
        self.subscription = self.create_subscription(
            PoseStamped,
            '/object_pose',
            self.listener_callback,
            10)
        self.cmd_pub = self.create_publisher(PosCmd, "/pos_cmd", 10)
        self.subscription_cmd = self.create_subscription(
        Int16,
        '/grab_ctrl',
        self.grab_callback,
        10)
        self.subscription  # prevent unused variable warning
        self.setCmd(x=0, y=0.4, z=0.4, roll=-np.pi/2, pitch=0.0, yaw=0.0, gripper=0.1)
        self.get_logger().info("Init pos")



        
    def grab_callback(self, msg):
        global grab_state
        if msg.data == 1:
            grab_state = True

        
    # def angleGet(self, radInput, multiInput):
    #     getAngle = int()

    def listener_callback(self, msg):
        # self.setCmd(x=0.3, y=0.0, z=0.3, roll=0.0, pitch=2.9, yaw=0.0, gripper=0.0)
        # exit(-1)
        start_time = time.time()
        global target_x, target_y, target_z, target_angle, grab_state, object_type
        # target_frame = 'world'
        target_frame = 'arm_base'
        stamp = rclpy.time.Time.from_msg(msg.header.stamp)
        self.tf_buffer.can_transform(target_frame, msg.header.frame_id, rclpy.time.Time())
        target_pose = self.tf_buffer.transform(msg, target_frame)
        
        
        
        # self.get_logger().info(f"Received object pose: {target_pose.pose.position.x}, {target_pose.pose.position.y}, {target_pose.pose.position.z}")

        rotation = R.from_quat([target_pose.pose.orientation.x, target_pose.pose.orientation.y, target_pose.pose.orientation.z, target_pose.pose.orientation.w])        
                
        z_axis = np.array([0, 0, 1])

      
        rotated_z_axis = rotation.apply(z_axis)

        
        print(f"ijk: {rotated_z_axis}")
        self.get_logger().warn(f"xyz: {target_pose.pose.position.x}, {target_pose.pose.position.y+0.04}, {target_pose.pose.position.z}")
        if rotated_z_axis[2] > 0.8 or rotated_z_axis[2] < -0.8:
            self.get_logger().warn("Object is vertical")
            return
        angle = math.atan(rotated_z_axis[1]/rotated_z_axis[0])/math.pi*180
        # if angle < 0:
        #     angle = 180 + angle
        # self.get_logger().info(f"Angle: {angle}")
        
        target_x = (-target_pose.pose.position.y + 0.19) * 0.88


  
        target_y = target_pose.pose.position.x + 0.27

           
        phase_angle = math.atan2(target_y+13, target_x)/math.pi*180   #3   x + ->
        print(f"Phase angle: {phase_angle}")
        print(f"Angle: {angle}")
        object_type = msg.header.stamp.sec

        
        self.get_logger().info(f"target_x: {target_x}, target_y: {target_y}, target_angle: {target_angle}")
        # time.sleep(1)
        if object_type == 0:
            target_x += 0.05
            target_y += 0.02
            
        
        if object_type == 2:
            target_x *= 0.895
            target_y += 0.05 
            
        
        # rz = R.from_euler('z', 0, degrees=False)
        angle_obj = np.pi/2 - ((angle)/180.0)*np.pi
        # if object_type == 2:
        #     angle_obj += np.pi/2
            
            
        if angle_obj < 0:
            angle_obj += np.pi
        if angle_obj > np.pi:
            angle_obj -= np.pi
        

        rz = R.from_euler('z', angle_obj, degrees=False)
        rx = R.from_euler('x', -np.pi * 0.9 , degrees=False)
        rxx = R.from_euler('x', -np.pi , degrees=False)
        r = rx * rz
        eular = r.as_euler('xyz', degrees=False)
        rr = rxx * rz
        eular_down = rr.as_euler('xyz', degrees=False)
        # eular[1] += np.pi/2
        print(f"eular: {eular}")
        if target_x < 0:
            target_x = target_x + 0.05    
        if target_x > 0:
            target_x = target_x - 0.06      
        
        print(f"target_pose.pose.position.x:{target_pose.pose.position.x}")
        print(f"target_pose.pose.position.y:{target_pose.pose.position.y}")
        print(f"target_y:{target_y}")
        print(f"target_x:{target_x}")
        time.sleep(2)
        
        # self.setCmd(x=target_y, y=target_x, z=0.3, roll=eular[0], pitch=eular[1], yaw=eular[2], gripper=0.05)
        self.setCmd(x=target_x, y=target_y, z=0.23, roll=eular[0], pitch=eular[1], yaw=eular[2], gripper=0.1)
        
        # self.setCmd(x=target_y, y=target_x, z=0.21, roll=eular[0], pitch=0.0, yaw=eular[2], gripper=0.1) #???????????


        
        # data = json.dumps({"T":1,"P6":10, "S6": 1000})
        # ser.write(data.encode())
        #TODO:
        # self.setCmd(x=0.3, y=0.0, z=0.3, roll=0.0, pitch=2.9, yaw=0.0, gripper=0.0)
        
        # self.get_logger().info("0")
        # time.sleep(2)
        
        
        # data = json.dumps({'T':2,'P1':target_x,'P2':target_y,'P3':186.5822754,'P4':180,'P5':target_angle,"S1":10,"S5":1000})
        # ser.write(data.encode())
        # self.setCmd(x=target_y, y=target_x, z=0.21, roll=eular[0], pitch=eular[1], yaw=eular[2], gripper=0.1)
        #TODO:
        self.get_logger().info("1")
    
        time.sleep(2) 
        # exit(-1)

        # if object_type == 2:
        #     data = json.dumps({'T':2,'P1':target_x,'P2':target_y,'P3':76.5822754,'P4':180,'P5':target_angle,"S1":10,"S5":1000})
        # else:
        
        # data = json.dumps({'T':2,'P1':target_x,'P2':target_y,'P3':106.5822754,'P4':180,'P5':target_angle,"S1":10,"S5":1000})
        #TODO:
        self.setCmd(x=target_x, y=target_y, z=0.17, roll=eular_down[0], pitch=eular_down[1], yaw=eular_down[2], gripper=0.1)
        # if object_type == 2:    
        #     self.setCmd(x=target_y, y=target_x, z=0.15, roll=eular[0], pitch=eular[1], yaw=eular[2], gripper=0.1)
            # data = json.dumps({'T':2,'P1':target_x,'P2':target_y,'P3':90.5822754,'P4':180,'P5':target_angle,"S1":10,"S5":1000})  
        #TODO:      
        # send_data = ser.write(data.encode())
        # print(f"send_data: {send_data}")
        self.get_logger().info("2")
        time.sleep(2)
       
        # data = json.dumps({"T":1,"P6":75, "S6": 1000})
        if object_type == 0:
            # data = json.dumps({"T":1,"P6":75, 'P3':116.5822754,"S6": 1000})
            self.setCmd(x=target_x, y=target_y, z=0.17, roll=eular_down[0], pitch=eular_down[1], yaw=eular_down[2], gripper=0.055)
            time.sleep(1)
            
            self.setCmd(x=target_x, y=target_y, z=0.28, roll=eular[0], pitch=eular[1], yaw=eular[2], gripper=0.055)
            time.sleep(1)
            
        #TODO:    
        
        elif object_type == 1:
            # data = json.dumps({"T":1,"P6":80, "S6": 1000})
            self.setCmd(x=target_x, y=target_y, z=0.17, roll=eular_down[0], pitch=eular_down[1], yaw=eular_down[2], gripper=0.055)
            time.sleep(1)
            
            self.setCmd(x=target_x, y=target_y, z=0.28, roll=eular[0], pitch=eular[1], yaw=eular[2], gripper=0.055)
            time.sleep(1)
            
        #TODO:
        elif object_type == 2:
            # data = json.dumps({"T":1,'P2':target_x-320, 'P3':106.5822754,"P6":70, "S6": 1000})
            self.setCmd(x=target_x, y=target_y, z=0.17, roll=eular_down[0], pitch=eular_down[1], yaw=eular_down[2], gripper=0.036)
            time.sleep(1)
            self.setCmd(x=target_x, y=target_y, z=0.28, roll=eular[0], pitch=eular[1], yaw=eular[2], gripper=0.036)
            time.sleep(1)
            
            
        #TODO:
        # exit(2)
        # send_data = ser.write(data.encode())
        # print(f"send_data: {send_data}")
        self.get_logger().info("3")

        #TODO:
        # data = json.dumps({'T':2,'P1':target_x,'P2':target_y,'P3':286.5822754,'P4':180,'P5':target_angle,"S1":10,"S5":1000})

        # send_data = ser.write(data.encode())
        # print(f"send_data: {send_data}")
        self.get_logger().info("4")

    
        if object_type == 0:
            #TODO:
            self.setCmd(x=0.42, y=-0.28, z=0.3, roll=0.0, pitch=1.8, yaw=0, gripper=0.055)
            # ser.write(data.encode())
            self.get_logger().info("6")
            time.sleep(2)
            self.setCmd(x=0.42, y=-0.28, z=0.3, roll=0.0, pitch=1.8, yaw=0, gripper=0.2)
            time.sleep(1)
            # exit(0)
        elif object_type == 1:
            #TODO:
            self.setCmd(x=0.42, y=0.0, z=0.3, roll=0.0, pitch=1.8, yaw=0, gripper=0.055)
            # x: 0.4, y: 0, z: 0.2, roll: 0, pitch: 1.8, yaw: 0.0, gripper: 0.055, mode1: 0, mode2: 0
            time.sleep(2)
            # ser.write(data.encode())
            self.get_logger().info("5")
            self.setCmd(x=0.42, y=0.0, z=0.3, roll=0.0, pitch=1.8, yaw=0, gripper=0.2)
            time.sleep(1)
            
        elif object_type == 2:
            #TODO:
            self.setCmd(x=0.42, y=0.28, z=0.3, roll=0.0, pitch=1.8, yaw=0, gripper=0.036)
            time.sleep(2)
            # ser.write(data.encode())
            self.get_logger().info("5")
            self.setCmd(x=0.42, y=0.28, z=0.3, roll=0.0, pitch=1.8, yaw=0, gripper=0.2)
            time.sleep(1)
        #TODO:
        data = json.dumps({"T":1,"P6":10, "S6": 1000})
        # ser.write(data.encode())
        self.get_logger().info("6")
        #TODO:
        data = json.dumps({'T':2,'P1':180.5104065,'P2':-13.75,'P3':186.5822754,'P4':180,'P5':235,"S1":10,"S5":1000})
        # ser.write(data.encode())
        self.get_logger().info("7")
        time.sleep(1)
        print(f"Total process time: {time.time() - start_time:.3f} seconds")
            


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
