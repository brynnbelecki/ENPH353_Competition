#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from rosgraph_msgs.msg import Clock

class DronePIDController:
    def __init__(self, namespace="D1"):
        self.ns = namespace
        rospy.init_node(f'{self.ns}_pid_controller', anonymous=True)

        rospy.set_param('/use_sim_time', True)

        self.pub_cmd = rospy.Publisher(f'drone_linear_cmd', Twist, queue_size=10)
        self.sub_depth = rospy.Subscriber(f'depth/image_raw', Image, self.depth_callback)
        self.sub_clock = rospy.Subscriber('/clock', Clock, self.clock_callback)

        # PID gains for altitude
        self.Kp_z = 200.0
        self.Ki_z = 25
        self.Kd_z = 250.0

        # PID gains for X/Y centering
        self.Kp_xy = 9
        self.Ki_xy = 8
        self.Kd_xy = 5

        # PID state
        self.prev_error_z = 0.0
        self.integral_z = 0.0
        self.prev_error_x = 0.0
        self.integral_x = 0.0
        self.prev_error_y = 0.0
        self.integral_y = 0.0

        # Target height
        self.target_height = 17

        self.bridge = CvBridge()
        self.current_time = None
        self.start_time = None
        self.active = False

        # Initial thrust
        self.initial_thrust_duration = 0.25
        self.initial_thrust_done = False
        self.initial_thrust_value = 600.0
        self.hover_baseline = 196.3

        self.min_thrust = -300
        self.max_thrust = 1000
        self.reached_height = False

    def clock_callback(self, msg: Clock):
        self.current_time = msg.clock.to_sec()
        if self.start_time is None:
            self.start_time = self.current_time

        if not self.active and self.current_time - self.start_time > 5:
            self.active = True
            self.thrust_start_time = self.current_time
            rospy.loginfo("[PID CTRL] Activated. Initial upward thrust starting!")

    def depth_callback(self, msg: Image):
        if self.current_time is None or not self.active:
            return

        dt_activation = self.current_time - self.thrust_start_time
        cmd = Twist()

        if not self.initial_thrust_done and dt_activation <= self.initial_thrust_duration:
            cmd.linear.z = self.initial_thrust_value
            self.pub_cmd.publish(cmd)
            rospy.loginfo(f"[INIT THRUST] linear.z = {cmd.linear.z:.2f}")
            return
        elif dt_activation > self.initial_thrust_duration:
            self.initial_thrust_done = True

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except CvBridgeError as e:
            rospy.logwarn(f"[DEPTH] CV bridge error: {e}")
            return

        # Depth mask for valid object
        mask = (cv_image > 0.005) & (cv_image < 25.0)
        if not np.any(mask):
            rospy.logwarn("[DEPTH] No valid depth")
            return

        height_estimate = float(np.mean(cv_image[mask]))
        if np.isnan(height_estimate) or height_estimate < 0.0001 or height_estimate > 25.0:
            rospy.logwarn(f"[DEPTH] Invalid depth: {height_estimate}")
            return

        # --- Compute object center ---
        ys, xs = np.where(mask)
        cx_obj = np.mean(xs)
        cy_obj = np.mean(ys)
        cx_img = cv_image.shape[1] // 2
        cy_img = cv_image.shape[0] // 2

        dx = cx_img - cx_obj 
        dy = cy_obj - cy_img

        error_x = np.sign(dx) * np.sqrt(abs(dx))
        error_y = np.sign(dy) * np.sqrt(abs(dy))  
        error_x = dx
        error_y = dy

        dt = 0.025  # timestep

        # --- PID for X ---
        if self.reached_height == True:
            self.integral_x += error_x * dt
        else: self.integral_x = 0
        derivative_x = (error_x - self.prev_error_x) / dt
        vx_cmd = self.Kp_xy * error_x + self.Ki_xy * self.integral_x + self.Kd_xy * derivative_x

        # --- PID for Y ---
        if self.reached_height == True:
            self.integral_y += error_y * dt
        else: self.integral_y = 0
        derivative_y = (error_y - self.prev_error_y) / dt
        vy_cmd = self.Kp_xy * error_y + self.Ki_xy * self.integral_y + self.Kd_xy * derivative_y

        # --- PID for Z ---
        error_z = self.target_height - height_estimate
        self.integral_z += error_z * dt
        derivative_z = (error_z - self.prev_error_z) / dt
        vz_cmd = self.hover_baseline + self.Kp_z * error_z + self.Ki_z * self.integral_z + self.Kd_z * derivative_z

        vz_cmd = np.clip(vz_cmd, self.min_thrust, self.max_thrust)
        #vx_cmd = np.clip(vx_cmd, -250.0, 250.0)  # max speed in X, adjust as needed
        #vy_cmd = np.clip(vy_cmd, -250.0, 250.0)

        cmd.linear.x = -10
        cmd.linear.y = -5
        if error_z < 0 or self.reached_height == True:
            self.reached_height = True
            cmd.linear.x = vx_cmd
            cmd.linear.y = vy_cmd
        cmd.linear.z = vz_cmd

        self.pub_cmd.publish(cmd)

        rospy.loginfo(f"[PID] Depth={height_estimate:.2f} m, "
                      f"ErrorX={error_x}, ErrorY={error_y}, ErrorZ={error_z:.2f}, "
                      f"Cmd=({vx_cmd:.3f},{vy_cmd:.3f},{vz_cmd:.2f})")

        self.prev_error_x = error_x
        self.prev_error_y = error_y
        self.prev_error_z = error_z


if __name__ == "__main__":
    drone_ns = rospy.get_param("~namespace", "D1")
    controller = DronePIDController(namespace=drone_ns)
    rospy.spin()
