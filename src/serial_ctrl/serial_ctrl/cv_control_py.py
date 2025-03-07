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


from rclpy.node import Node
import socket
from scipy.spatial.transform import Rotation 
from scipy.spatial.transform import Rotation as R
import numpy as np
import math
from tf2_ros import TransformException
ser = serial.Serial("/dev/ttyUSB0",115200)
object_type = {"bottle","can","carton"}
stand_state = None
target_x = 180.5104065
target_y = -13.75
target_z = 186.5822754
target_angle = 235
grab_state = False
object_type = -1
class MinimalSubscriber(Node):

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
        self.subscription_cmd = self.create_subscription(
        Int16,
        '/grab_ctrl',
        self.grab_callback,
        10)
        self.subscription  # prevent unused variable warning
        # self.subscription_cmd
        # {"T":2,"P1":300.5104065,"P2":-13.75,"P3":156.5822754,"P4":180,"P5":235,"S1":10,"S5":200}
        data = json.dumps({'T':2,'P1':180.5104065,'P2':-13.75,'P3':186.5822754,'P4':180,'P5':235,"S1":10,"S5":1000})
        ser.write(data.encode())
        # self.get_logger().info('Subscribing to /object_pose')
        # time.sleep(3)
        # data = json.dumps({"T":1,"P6":40})
        # ser.write(data.encode())
        # self.get_logger().info('Subscribing to /grab_ctrl')
        # time.sleep(3)

        
    def grab_callback(self, msg):
        global grab_state
        if msg.data == 1:
            grab_state = True
            # time.sleep(2)

            # data = json.dumps({'T':2,'P1':target_x,'P2':target_y,'P3':106.5822754,'P4':180,'P5':target_angle,"S1":10,"S5":1000})
            # ser.write(data.encode())
            # time.sleep(1.5)
            
            # # data = json.dumps({"T":1,"P6":75, "S6": 1000})
            # if object_type == 0:
            #     data = json.dumps({"T":1,"P6":70, "S6": 1000})
            # elif object_type == 1:
            #     data = json.dumps({"T":1,"P6":80, "S6": 1000})
            # elif object_type == 2:
            #     data = json.dumps({"T":1,"P6":100, "S6": 1000})
            
            # ser.write(data.encode())
            # time.sleep(1.5)
            
            # data = json.dumps({'T':2,'P1':target_x,'P2':target_y,'P3':186.5822754,'P4':180,'P5':target_angle,"S1":10,"S5":1000})
            # ser.write(data.encode())
            # time.sleep(1.5)
            
            # data = json.dumps({'T':2,'P1':60.5104065,'P2':-373.75,'P3':186.5822754,'P4':180,'P5':235,"S1":10,"S5":1000})
            # ser.write(data.encode())
            # time.sleep(7)
            
            # data = json.dumps({"T":1,"P6":40, "S6": 1000})
            # ser.write(data.encode())
            # time.sleep(1.5)
            
            # data = json.dumps({'T':2,'P1':180.5104065,'P2':-13.75,'P3':186.5822754,'P4':180,'P5':235,"S1":10,"S5":1000})
            # ser.write(data.encode())
            # time.sleep(5)
            
            # grab_state = False
            
        # elif msg.data == 0:
        #     data = json.dumps({"T":1,"P6":40, "S6": 1000})
        #     ser.write(data.encode())
        #     time.sleep(3)
        #     data = json.dumps({'T':2,'P1':180.5104065,'P2':-13.75,'P3':186.5822754,'P4':180,'P5':235,"S1":10,"S5":1000})
        #     ser.write(data.encode())
        #     time.sleep(3)
        #     grab_state = False
        
    # def angleGet(self, radInput, multiInput):
    #     getAngle = int()

    def listener_callback(self, msg):
        global target_x, target_y, target_z, target_angle, grab_state, object_type
        # target_frame = 'world'
        target_frame = 'arm_base'
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
        
        target_x = int(180.5104065 +( -0.14 + target_pose.pose.position.x) * 1000)

  
        target_y = int(-14.75 - (target_pose.pose.position.y) * 1000 * 0.55)

           
        phase_angle = math.atan2(target_y+13, target_x)/math.pi*180
        print(f"Phase angle: {phase_angle}")
        print(f"Angle: {angle}")
        target_angle = int(235 + angle + phase_angle)
        if target_angle < 105:
            target_angle = 145
        if target_angle > 375:
            target_angle = 375
          

        object_type = msg.header.stamp.sec

        
        if target_y > 180:
            target_y = 180
        if target_y < -180:
            target_y = -180
        if target_x > 300:
            target_x = 300
        if target_x < 180:
            target_x = 180
        self.get_logger().info(f"target_x: {target_x}, target_y: {target_y}, target_angle: {target_angle}")
        # time.sleep(1)
        data = json.dumps({'T':2,'P1':target_x,'P2':target_y,'P3':186.5822754,'P4':180,'P5':target_angle,"S1":10,"S5":1000})
        ser.write(data.encode())
        self.get_logger().info("1")
    
        time.sleep(3)

        # if object_type == 2:
        #     data = json.dumps({'T':2,'P1':target_x,'P2':target_y,'P3':76.5822754,'P4':180,'P5':target_angle,"S1":10,"S5":1000})
        # else:
        data = json.dumps({'T':2,'P1':target_x,'P2':target_y,'P3':106.5822754,'P4':180,'P5':target_angle,"S1":10,"S5":1000})
            
            
        ser.write(data.encode())
        self.get_logger().info("2")
        time.sleep(3)
        
        # data = json.dumps({"T":1,"P6":75, "S6": 1000})
        # if object_type == 0:
        #     data = json.dumps({"T":1,"P6":70, "S6": 1000})
        # elif object_type == 1:
        data = json.dumps({"T":1,"P6":80, "S6": 1000})
        # elif object_type == 2:
        #     print("carton")
        #     data = json.dumps({"T":1,"P6":65, "S6": 1000})
        
        ser.write(data.encode())
        self.get_logger().info("3")
        time.sleep(3)
        
        data = json.dumps({'T':2,'P1':target_x,'P2':target_y,'P3':186.5822754,'P4':180,'P5':target_angle,"S1":10,"S5":1000})
        ser.write(data.encode())
        self.get_logger().info("4")
        time.sleep(3)
        
        data = json.dumps({'T':2,'P1':60.5104065,'P2':-373.75,'P3':186.5822754,'P4':180,'P5':235,"S1":10,"S5":1000})
        ser.write(data.encode())
        self.get_logger().info("5")
        time.sleep(7)
        
        data = json.dumps({"T":1,"P6":10, "S6": 1000})
        ser.write(data.encode())
        self.get_logger().info("6")
        time.sleep(2)
        
        data = json.dumps({'T':2,'P1':180.5104065,'P2':-13.75,'P3':186.5822754,'P4':180,'P5':235,"S1":10,"S5":1000})
        ser.write(data.encode())
        self.get_logger().info("7")
        time.sleep(5)
            


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
