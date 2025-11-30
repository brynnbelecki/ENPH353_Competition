#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
import tf.transformations as tft
import math
import numpy as np

class YawPController:
    def __init__(self, namespace):
        self.ns = namespace
        rospy.init_node(f"{self.ns}_yaw_p_controller", anonymous=True)

        # Publisher for yaw angular velocity command
        self.pub_cmd = rospy.Publisher(f"/{self.ns}/drone_angular_cmd", Twist, queue_size=10)

        # IMU -> get current yaw
        self.sub_imu = rospy.Subscriber(f"/{self.ns}/imu/data", Imu, self.imu_callback)

        # Target yaw in radians
        self.sub_target = rospy.Subscriber("/drone_target", Float32MultiArray, self.target_callback)

        self.Kp = 5.0   # proportional gain
        self.yaw_setpoint = -np.pi/2

    def target_callback(self, msg: Float32MultiArray):
        self.yaw_setpoint = msg.data[3]

    def imu_callback(self, msg: Imu):
        
        q = msg.orientation
        roll, pitch, yaw = tft.euler_from_quaternion([q.x, q.y, q.z, q.w])
        
        # Normalize yaw to [-pi, pi]
        yaw = (yaw + math.pi) % (2 * math.pi) - math.pi

        # Compute yaw error
        error = self.yaw_setpoint - yaw

        # Wrap error to [-pi, pi]
        error = (error + math.pi) % (2 * math.pi) - math.pi

        # P-control output
        cmd = Twist()

        cmd.angular.z = self.Kp * error



        self.pub_cmd.publish(cmd)


if __name__ == "__main__":
    ns = rospy.get_param("~namespace", "D2")
    controller = YawPController(namespace=ns)
    rospy.spin()
