#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32MultiArray
import numpy as np

class DroneTargetTrigger:
    def __init__(self):
        rospy.init_node("drone_target_trigger")
        self.pub = rospy.Publisher("/drone_target", Float32MultiArray, queue_size=10)
        rospy.Subscriber("/drone_coords", Float32MultiArray, self.coords_callback)

        # List of target coordinates [x, y, z, yaw]
        self.targets = [
            [0.04, 0.875, 1, 2.25],           
            [0.07, 0.29, 1, -3.1415/2],
            [0.145, 0.2, 1, 3.1415],
            [0.45, 0.325, 1, 0.9],
            [0.425, 0.8, 1, -3.1415/2],
            [0.775, 0.8, 1, 3.1415]
        ]

        # Error threshold for x/y to trigger z change
        self.xy_threshold = 0.01  # meters
        self.threshold_count_required = 5  # number of consecutive cycles

        # Current drone position
        self.curr_x = 0.0
        self.curr_y = 0.0

        # Start main loop
        self.rate = rospy.Rate(50)  # 10 Hz
        self.main_loop()

    def coords_callback(self, msg: Float32MultiArray):
        if len(msg.data) < 4:
            return
        # Average corners to get center
        self.curr_x = (msg.data[0] + msg.data[2]) / 2
        self.curr_y = (msg.data[1] + msg.data[3]) / 2

    def main_loop(self):
        while not rospy.is_shutdown():
            for target in self.targets:
                tx, ty, tz_target, yaw = target

                # --- Step 1: Go to (x, y) at z=1 ---
                self.publish_target(tx, ty, 1.0, yaw)
                rospy.loginfo(f"Heading to target XY: [{tx:.3f}, {ty:.3f}] at Z=1")

                # Wait until drone reaches XY threshold for N consecutive calls
                consecutive_count = 0
                while not rospy.is_shutdown():
                    error = np.sqrt((tx - self.curr_x) ** 2 + (ty - self.curr_y) ** 2)
                    if error < self.xy_threshold:
                        consecutive_count += 1
                        if consecutive_count >= self.threshold_count_required:
                            break
                    else:
                        consecutive_count = 0  # reset if outside threshold
                    self.publish_target(tx, ty, 1.0, yaw)
                    self.rate.sleep()

                self.publish_target(tx, ty, 0.0, yaw)
                rospy.sleep(0.5)

                self.publish_target(tx, ty, 1.0, yaw)
                rospy.sleep(0.25)


    def publish_target(self, x, y, z, yaw):
        msg = Float32MultiArray()
        msg.data = [x, y, z, yaw]
        self.pub.publish(msg)


if __name__ == "__main__":
    try:
        DroneTargetTrigger()
    except rospy.ROSInterruptException:
        pass

