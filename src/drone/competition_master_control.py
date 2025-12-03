#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32MultiArray
import numpy as np

class DroneTargetTrigger:
    def __init__(self):
        rospy.init_node("drone_target_trigger")
        self.pub = rospy.Publisher("/drone_target", Float32MultiArray, queue_size=10)
        rospy.Subscriber("/drone_coords", Float32MultiArray, self.coords_callback)

        self.targets = [
            [0.008, 0.84, 0.5, -1/2],   # sign 1          
            [0.064, 0.29, 0.5, -1/2],    # sign 2
            [0.145, 0.2, 0.5, 1],        # sign 3  
            [0.43, 0.37, 0.5, 1/2],      # sign 4
            [0.43, 0.82, 0.5, -1/2],     # sign 5  
            [0.76, 0.82, 0.5, 1],        # sign 6  
            [0.835, 0.12, 0.5, 0],       # sign 7  
            [0.85, 0.12, 2.5, 0],      # waypoint (special: no Z=0 drop, just wait)
            [0.63, 0.27, 2.5, 0], # waypoint (special: no Z=0 drop, just wait)
            [0.63, 0.27, 1, 0],        # final sign
            [0.75, 0.1, 2.5, 0], #slam into the tunnel
            [0.75, 0.1, 0, 0]
        ]

        self.xy_threshold = 0.005  
        self.threshold_count_required = 15

        self.curr_x = 0.0
        self.curr_y = 0.0
        self.curr_z = 0.0

        self.rate = rospy.Rate(50)
        self.main_loop()

    def coords_callback(self, msg: Float32MultiArray):
        if len(msg.data) < 4:
            return

        self.curr_x = (msg.data[0] + msg.data[2]) / 2
        self.curr_y = (msg.data[1] + msg.data[3]) / 2


    def main_loop(self):
        while not rospy.is_shutdown():

            for target in self.targets:
                tx, ty, tz, yaw = target

                is_waypoint = (tz > 1.5)

                self.publish_target(tx, ty, tz, yaw)
                rospy.loginfo(f"Heading to target XY: [{tx:.3f}, {ty:.3f}] Z={tz}")

                consecutive_count = 0

                if is_waypoint:
                    rospy.sleep(0.5)
                    continue  

                while not rospy.is_shutdown():

                    xy_error = np.sqrt((tx - self.curr_x)**2 + (ty - self.curr_y)**2)

                    if xy_error < self.xy_threshold:
                        consecutive_count += 1
                        if consecutive_count >= self.threshold_count_required:
                            break
                    else:
                        consecutive_count = 0

                    self.publish_target(tx, ty, tz, yaw)
                    self.rate.sleep()

                self.publish_target(tx, ty, 0.0, yaw)
                # just need the sign reading call here, it should be looking right at the sign 
                rospy.sleep(0.5)

                self.publish_target(tx, ty, tz, yaw)
                rospy.sleep(0.5)
            return

    def publish_target(self, x, y, z, yaw):
        msg = Float32MultiArray()
        msg.data = [x, y, z, yaw * np.pi]
        self.pub.publish(msg)


if __name__ == "__main__":
    try:
        DroneTargetTrigger()
    except rospy.ROSInterruptException:
        pass
