import rclpy
from rclpy.node import Node
import array

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped
import tf2_ros
import json
import serial
import tf2_geometry_msgs

ser = serial.Serial("/dev/ttyUSB0",115200)
j_mode = 0
object_type = {"bottle","can","carton"}
stand_state = None

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
        self.subscription  # prevent unused variable warning
    
        
    # def angleGet(self, radInput, multiInput):
    #     getAngle = int()

    def listener_callback(self, msg):
        target_frame = 'arm_base1'
        self.tf_buffer.can_transform(target_frame, msg.header.frame_id, rclpy.time.Time(), rclpy.duration.Duration(seconds=1.0))
        target_pose = self.tf_buffer.transform(msg, target_frame)
        self.get_logger().info(f"Received object pose: {0.66-target_pose.pose.position.x}, {target_pose.pose.position.y + 0.04}, {target_pose.pose.position.z}")
        #{"T":2,"P1":270.5104065,"P2":-13.75,"P3":86.5822754,"P4":90,"P5":235,"S1":10,"S5":200}
        target_x = 270.5104065 +( 0.66 - target_pose.pose.position.x) * 1000
        if target_x > 350:
            target_x = 350
        if target_x < 250:
            target_x = 250
        target_y = -13.75 + (target_pose.pose.position.y + 0.04) * 1000
        data = json.dumps({'T':2,'P1':target_x,'P2':-13.75,'P3':86.5822754,'P4':90,'P5':235,"S1":10,"S5":200})
        print(data)
        ser.write(data.encode())


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
