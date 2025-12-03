#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Image
from rosgraph_msgs.msg import Clock
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import threading
from std_msgs.msg import Bool

class DronePIDController:
    def __init__(self, namespace="D2"):
        self.ns = namespace
        rospy.init_node(f'{self.ns}_pid_controller', anonymous=True)
        rospy.set_param('/use_sim_time', True)

        # Publishers / Subscribers
        self.pub_cmd = rospy.Publisher("drone_linear_cmd", Twist, queue_size=10)
        rospy.Subscriber("/drone_coords", Float32MultiArray, self.coords_callback)
        rospy.Subscriber("depth/image_raw", Image, self.depth_callback)
        rospy.Subscriber("/clock", Clock, self.clock_callback)
        rospy.Subscriber("/drone_target", Float32MultiArray, self.target_callback)
        rospy.Subscriber("/pid_tuning", Float32MultiArray, self.pid_tuning_callback)

        self.bridge = CvBridge()

        self.Kp_z = 200.0
        self.Ki_z = 25
        self.Kd_z = 200.0
        self.Kp_xy = 12000
        self.Ki_xy = 10000
        self.Kd_xy = 4500
        self.y_scale = 0.475

        self.prev_error_z = 0.0
        self.prev_error_x = 0.0
        self.prev_error_y = 0.0
        self.integral_z = 0.0
        self.integral_x = 0.0
        self.integral_y = 0.0

        self.hover_baseline = 198.6
        self.min_thrust = -150000
        self.max_thrust = 150000

        self.target_x = 0.05
        self.target_y = 0.825
        self.target_z = 0.5
        self.new_target = True

        self.curr_x = 0.1
        self.curr_y = 0.9
        self.curr_z = 0.0

        self.deriv_window = 10
        self.dx_history = []
        self.dy_history = []

        self.current_time = None
        self.start_time = None
        self.start_delay = 7

        self._pid_thread = threading.Thread(target=self.pid_loop)
        self._pid_thread.daemon = True
        self._pid_thread.start()

    def clock_callback(self, msg: Clock):
        self.current_time = msg.clock.to_sec()
        if self.start_time is None:
            self.start_time = self.current_time
            rospy.loginfo("[PID CTRL] Activated. Applying initial downward thrust!")

    def target_callback(self, msg: Float32MultiArray):
        if len(msg.data) < 3:
            return
        self.target_x = msg.data[0]
        self.target_y = msg.data[1]
        self.target_z = msg.data[2]
        self.new_target = True

    def coords_callback(self, msg: Float32MultiArray):
        if len(msg.data) < 4 or (msg.data[0] == -1 and msg.data[1] == -1):
            return
        self.curr_x = (msg.data[0] + msg.data[2]) / 2
        self.curr_y = (msg.data[1] + msg.data[3]) / 2

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
            self.curr_z = 0.0
            return
        self.curr_z = float(np.max(depth_image[mask]))

    def smooth_derivative(self, history, new_val):
        history.append(new_val)
        if len(history) > self.deriv_window:
            history.pop(0)
        return sum(history) / len(history)

    def pid_tuning_callback(self, msg: Float32MultiArray):
        if len(msg.data) >= 6:
            self.Kp_xy, self.Ki_xy, self.Kd_xy, self.Kp_z, self.Ki_z, self.Kd_z = msg.data
            rospy.loginfo(f"Updated PID gains: Kp_xy={self.Kp_xy}, Ki_xy={self.Ki_xy}, Kd_xy={self.Kd_xy}, "
                          f"Kp_z={self.Kp_z}, Ki_z={self.Ki_z}, Kd_z={self.Kd_z}")

    def pid_loop(self):
        rate = rospy.Rate(400)  
        dt = 1.0 / 400.0
        while not rospy.is_shutdown():
            if self.current_time is None:
                rate.sleep()
                continue

            if self.current_time - self.start_time < self.start_delay:
                cmd = Twist()
                cmd.linear.z = -5000.0
                self.new_target = True
                self.pub_cmd.publish(cmd)
                rate.sleep()
                continue


            self.publish_pid(dt)
            rate.sleep()

    def publish_pid(self, dt):
        cmd = Twist()

        error_x = -(self.target_x - self.curr_x)
        error_x = np.sign(error_x) * abs(error_x)
        error_y = (self.target_y - self.curr_y)
        error_y = np.sign(error_y) * abs(error_y) * self.y_scale
        error_z = self.target_z - self.curr_z
        

        self.integral_x += error_x * dt
        self.integral_y += error_y * dt
        self.integral_z += error_z * dt

        raw_dx = (error_x - self.prev_error_x) / dt
        raw_dy = (error_y - self.prev_error_y) / dt
        derivative_x = self.smooth_derivative(self.dx_history, raw_dx)
        derivative_y = self.smooth_derivative(self.dy_history, raw_dy)
        derivative_z = (error_z - self.prev_error_z) / dt

        if self.new_target:
            derivative_x = 0
            derivative_y = 0
            derivative_z = 0
            self.new_target = False

        

        vx_cmd = self.Kp_xy * error_x + self.Ki_xy * self.integral_x + self.Kd_xy * derivative_x
        vy_cmd = self.Kp_xy * error_y + self.Ki_xy * self.integral_y + self.Kd_xy * derivative_y
        vz_cmd = self.hover_baseline + (
            self.Kp_z * error_z + self.Ki_z * self.integral_z + self.Kd_z * derivative_z
        )
        vz_cmd = np.clip(vz_cmd, self.min_thrust, self.max_thrust)
        vx_cmd = np.clip(vx_cmd, self.min_thrust, self.max_thrust)
        vy_cmd = np.clip(vy_cmd, self.min_thrust, self.max_thrust)

        if self.target_z < 0.1:
            vz_cmd = -5000

        cmd.linear.x = vx_cmd
        cmd.linear.y = vy_cmd
        cmd.linear.z = vz_cmd

        self.pub_cmd.publish(cmd)

        # --- Save previous errors ---
        self.prev_error_x = error_x
        self.prev_error_y = error_y
        self.prev_error_z = error_z



if __name__ == "__main__":
    controller = DronePIDController(namespace="D2")
    rospy.spin()
