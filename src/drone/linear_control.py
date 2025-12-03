#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from rosgraph_msgs.msg import Clock
import cv2


class DronePIDController:
    def __init__(self, namespace="D1"):
        self.ns = namespace
        rospy.init_node(f'{self.ns}_pid_controller', anonymous=True)

        rospy.set_param('/use_sim_time', True)

        # Publishers / Subscribers
        self.pub_cmd = rospy.Publisher("drone_linear_cmd", Twist, queue_size=10)
        self.sub_depth = rospy.Subscriber("depth/image_raw", Image, self.depth_callback)
        self.sub_clock = rospy.Subscriber("/clock", Clock, self.clock_callback)

        # PID gains
        self.Kp_z = 200.0
        self.Ki_z = 25
        self.Kd_z = 200.0

        N = 5

        self.Kp_xy = 12 * N
        self.Ki_xy = 15 * N
        self.Kd_xy = 1.5 * N
        self.y_scale = 1.05

        # PID memory
        self.prev_error_z = 0.0
        self.integral_z = 0.0
        self.prev_error_x = 0.0
        self.integral_x = 0.0
        self.prev_error_y = 0.0
        self.integral_y = 0.0

        # Derivative smoothing buffers
        self.deriv_hist_x = []
        self.deriv_hist_y = []
        self.deriv_hist_z = []
        self.deriv_window = 5  # average last 5 derivative terms

        # Target height
        self.target_height = 30.0

        # CV / Time
        self.bridge = CvBridge()
        self.current_time = None
        self.last_time = None

        # Activation state
        self.start_time = None
        self.active = False

        # Centroid smoothing
        self.cx_prev = None
        self.cy_prev = None
        self.smooth_alpha = 0.1  

        # Hover and thrust limits
        self.hover_baseline = 198.6
        self.min_thrust = -300
        self.max_thrust = 1000

    # ------------------------------------------------------------
    # Time Sync
    # ------------------------------------------------------------
    def clock_callback(self, msg: Clock):
        self.current_time = msg.clock.to_sec()
        if self.start_time is None:
            self.start_time = self.current_time
            self.last_time = self.current_time
            rospy.loginfo("[PID CTRL] Activated. Applying initial downward thrust!")

        if not self.active and (self.current_time - self.start_time > 5):
            self.active = True
            rospy.loginfo("[PID CTRL] PID control active.")

    # ------------------------------------------------------------
    # Depth Callback
    # ------------------------------------------------------------
    def depth_callback(self, msg: Image):
        if self.current_time is None:
            return

        dt = self.current_time - self.last_time
        if dt <= 0:
            dt = 0.02
        self.last_time = self.current_time

        cmd = Twist()

        # -------- Initial downward thrust for first 1.5s --------
        if not self.active:
            cmd.linear.z = -2000.0
            self.pub_cmd.publish(cmd)
            return

        # -------- Convert depth image --------
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except CvBridgeError as e:
            rospy.logwarn(f"[DEPTH] CV bridge error: {e}")
            return

        mask = (cv_image > 0.005) & (cv_image < 40.0)
        if not np.any(mask):
            height_estimate = 0.0
        else:
            height_estimate = float(np.median(cv_image[mask]))
            if np.isnan(height_estimate) or height_estimate <= 0 or height_estimate > 40:
                height_estimate = 0.0

        # Centroid calculation
        mask_uint8 = mask.astype(np.uint8)
        M = cv2.moments(mask_uint8)
        if M["m00"] == 0:
            cx_obj, cy_obj = (cv_image.shape[1]-1)/2.0, (cv_image.shape[0]-1)/2.0
        else:
            cx_obj = M["m10"] / M["m00"]
            cy_obj = M["m01"] / M["m00"]

        # Image center
        cx_img = (cv_image.shape[1] - 1) / 2.0
        cy_img = (cv_image.shape[0] - 1) / 2.0

        # Centroid smoothing
        if self.cx_prev is None:
            self.cx_prev = cx_obj
            self.cy_prev = cy_obj

        cx_obj = self.smooth_alpha * cx_obj + (1 - self.smooth_alpha) * self.cx_prev
        cy_obj = self.smooth_alpha * cy_obj + (1 - self.smooth_alpha) * self.cy_prev

        self.cx_prev = cx_obj
        self.cy_prev = cy_obj

        # Errors
        error_x = cx_img - cx_obj
        error_y = (cy_obj - cy_img) * self.y_scale
        error_z = self.target_height - height_estimate

        if abs(error_x) < 0.25:
            error_x = 0
        if abs(error_y) < 0.25:
            error_y = 0

        # --------------------------------------------------------
        # PID X with averaged derivative
        # --------------------------------------------------------
        self.integral_x += error_x * dt
        raw_dx = (error_x - self.prev_error_x) / dt
        self.deriv_hist_x.append(raw_dx)
        if len(self.deriv_hist_x) > self.deriv_window:
            self.deriv_hist_x.pop(0)
        derivative_x = np.mean(self.deriv_hist_x)
        vx_cmd = self.Kp_xy * error_x + self.Ki_xy * self.integral_x + self.Kd_xy * derivative_x

        # --------------------------------------------------------
        # PID Y with averaged derivative
        # --------------------------------------------------------
        self.integral_y += error_y * dt
        raw_dy = (error_y - self.prev_error_y) / dt
        self.deriv_hist_y.append(raw_dy)
        if len(self.deriv_hist_y) > self.deriv_window:
            self.deriv_hist_y.pop(0)
        derivative_y = np.mean(self.deriv_hist_y)
        vy_cmd = self.Kp_xy * error_y + self.Ki_xy * self.integral_y + self.Kd_xy * derivative_y

        # --------------------------------------------------------
        # PID Z with averaged derivative
        # --------------------------------------------------------
        self.integral_z += error_z * dt
        raw_dz = (error_z - self.prev_error_z) / dt
        self.deriv_hist_z.append(raw_dz)
        if len(self.deriv_hist_z) > self.deriv_window:
            self.deriv_hist_z.pop(0)
        derivative_z = np.mean(self.deriv_hist_z)

        vz_cmd = self.hover_baseline + (
            self.Kp_z * error_z +
            self.Ki_z * self.integral_z +
            self.Kd_z * derivative_z
        )
        vz_cmd = np.clip(vz_cmd, self.min_thrust, self.max_thrust)

        # Publish
        cmd.linear.x = vx_cmd
        cmd.linear.y = vy_cmd
        cmd.linear.z = vz_cmd

        self.pub_cmd.publish(cmd)

        # Store previous errors
        self.prev_error_x = error_x
        self.prev_error_y = error_y
        self.prev_error_z = error_z


if __name__ == "__main__":
    drone_ns = rospy.get_param("~namespace", "D1")
    controller = DronePIDController(namespace=drone_ns)
    rospy.spin()
