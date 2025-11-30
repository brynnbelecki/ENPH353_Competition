#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Image
from rosgraph_msgs.msg import Clock
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

class DronePIDController:
    def __init__(self, namespace="D2"):
        self.ns = namespace
        rospy.init_node(f'{self.ns}_pid_controller', anonymous=True)
        rospy.set_param('/use_sim_time', True)

        # Publishers / Subscribers
        self.pub_cmd = rospy.Publisher("drone_linear_cmd", Twist, queue_size=10)
        self.sub_coords = rospy.Subscriber("/drone_coords", Float32MultiArray, self.coords_callback)
        self.sub_depth = rospy.Subscriber("depth/image_raw", Image, self.depth_callback)
        self.sub_clock = rospy.Subscriber("/clock", Clock, self.clock_callback)
        self.sub_target = rospy.Subscriber("/drone_target", Float32MultiArray, self.target_callback)


        self.bridge = CvBridge()

        self.Kp_z = 200.0
        self.Ki_z = 25
        self.Kd_z = 200.0
        
        self.Kp_xy = 5000
        self.Ki_xy = 3000
        self.Kd_xy = 6000
        self.y_scale = 0.47

        self.prev_error_z = 0.0
        self.integral_z = 0.0
        self.prev_error_x = 0.0
        self.integral_x = 0.0
        self.prev_error_y = 0.0
        self.integral_y = 0.0


        self.hover_baseline = 198.6

        self.min_thrust = -10000
        self.max_thrust = 10000

        self.target_x = 0.25
        self.target_y = 0.5
        self.target_z = 1.25

        self.current_time = None
        self.last_time = None
        self.start_time = None
        self.start_delay = 3  # seconds before PID starts

        # Current drone coordinates
        self.curr_x = 0.1
        self.curr_y = 0.9
        self.curr_z = 0.0

        self.new_target = True



    # ---------------- Clock ----------------
    def clock_callback(self, msg: Clock):
        self.current_time = msg.clock.to_sec()
        if self.start_time is None:
            self.start_time = self.current_time

        if self.last_time is None:
            self.last_time = self.current_time


    def target_callback(self, msg: Float32MultiArray):
        if len(msg.data) < 3:
            return
        self.target_x = msg.data[0]
        self.target_y = msg.data[1]
        self.target_z = msg.data[2]
        self.new_target = True

    def coords_callback(self, msg: Float32MultiArray):
        if len(msg.data) < 4:
            return

        if msg.data[0] == -1 and msg.data[1] == -1:
            return

        self.curr_x = (msg.data[0] + msg.data[2])/2
        self.curr_y = (msg.data[1] + msg.data[3])/2

        if self.current_time - self.start_time >= self.start_delay:
            # Reset dt for first PID cycle
            if self.last_time < self.start_time + self.start_delay:
                self.last_time = self.current_time
            self.publish_pid()

    def depth_callback(self, msg: Image):
        if self.current_time is None:
            return

        try:
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except CvBridgeError as e:
            rospy.logwarn(f"[DEPTH] CV bridge error: {e}")
            return

        mask = (depth_image > 0.005) & (depth_image < 40.0)
        if not np.any(mask):
            rospy.logwarn("[DEPTH] No valid depth")
            self.curr_z = 0.0
            return

        self.curr_z = float(np.median(depth_image[mask]))

    def publish_pid(self):
        dt = self.current_time - self.last_time
        if dt <= 0:
            dt = 0.02
        self.last_time = self.current_time

        cmd = Twist()

        error_x = -(self.target_x - self.curr_x)

        error_y = (self.target_y - self.curr_y) * self.y_scale

        error_z = self.target_z - self.curr_z

        self.integral_x += error_x * dt
        derivative_x = (error_x - self.prev_error_x) / dt

        self.integral_y += error_y * dt
        derivative_y = (error_y - self.prev_error_y) / dt

        if self.new_target:
            derivative_x = 0
            derivative_y = 0
            self.new_target = False
   
        vx_cmd = self.Kp_xy * error_x + self.Ki_xy * self.integral_x + self.Kd_xy * derivative_x

        vy_cmd = self.Kp_xy * error_y + self.Ki_xy * self.integral_y + self.Kd_xy * derivative_y

        self.integral_z += error_z * dt
        derivative_z = (error_z - self.prev_error_z) / dt
        vz_cmd = self.hover_baseline + (
            self.Kp_z * error_z + self.Ki_z * self.integral_z + self.Kd_z * derivative_z
        )
        vz_cmd = np.clip(vz_cmd, self.min_thrust, self.max_thrust)

        cmd.linear.x = vx_cmd
        cmd.linear.y = vy_cmd 
        cmd.linear.z = vz_cmd
        self.pub_cmd.publish(cmd)


        self.prev_error_x = error_x
        self.prev_error_y = error_y
        self.prev_error_z = error_z



if __name__ == "__main__":
    controller = DronePIDController(namespace="D2")
    rospy.spin()
