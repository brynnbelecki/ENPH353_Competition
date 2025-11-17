#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
from rosgraph_msgs.msg import Clock
import tf.transformations as tft

class DronePIDController:
    def __init__(self, namespace="D1"):
        self.ns = namespace
        rospy.init_node(f'{self.ns}_pid_stabilizer', anonymous=True)

        rospy.set_param('/use_sim_time', True)

        self.pub_cmd = rospy.Publisher(f'/{self.ns}/drone_angular_cmd', Twist, queue_size=10)

        self.sub_imu = rospy.Subscriber(f'/{self.ns}/imu/data', Imu, self.imu_callback)
        self.sub_clock = rospy.Subscriber('/clock', Clock, self.clock_callback)

        self.roll_gains  = [15, 0, 2]
        self.pitch_gains = [5, 0, 2]
        self.yaw_gains   = [5, 0, 2]

        self.prev_error = {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0}
        self.integral   = {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0}
        self.last_time  = None

        self.setpoint = {'roll': 0.0, 'pitch': 0.0, 'yaw': -1.57}

        self.current_time = None

        self.active = False
        self.start_time = None

    def clock_callback(self, msg: Clock):
        self.current_time = msg.clock.to_sec()
        if self.start_time is None:
            self.start_time = self.current_time
        elif not self.active and self.current_time - self.start_time > 2.0:
            self.active = True
            rospy.loginfo("Drone PID orientation controller activated!")

    def imu_callback(self, msg: Imu):
        if not self.active or self.current_time is None:
            return  # wait until activation

        if self.last_time is None:
            self.last_time = self.current_time
            return

        dt = self.current_time - self.last_time
        if dt <= 0.0:
            return

        # Convert quaternion to Euler angles (roll, pitch, yaw)
        q = msg.orientation
        roll, pitch, yaw = tft.euler_from_quaternion([q.x, q.y, q.z, q.w])
        angles = {'roll': roll, 'pitch': pitch, 'yaw': yaw}

        cmd = Twist()

        # Orientation PID
        for axis in ['roll', 'pitch', 'yaw']:
            error = self.setpoint[axis] - angles[axis]
            self.integral[axis] += error * dt
            derivative = (error - self.prev_error[axis]) / dt
            Kp, Ki, Kd = getattr(self, f"{axis}_gains")
            output = Kp * error + Ki * self.integral[axis] + Kd * derivative
            self.prev_error[axis] = error

            if axis == 'roll':
                cmd.angular.x = output
            elif axis == 'pitch':
                cmd.angular.y = output
            else:
                cmd.angular.z = output

        #rospy.loginfo(f"[PID] Time: {self.current_time:.2f}, "
                   # f"Roll Error: {self.prev_error['roll']:.3f}, Pitch Error: {self.prev_error['pitch']:.3f}, Yaw Error: {self.prev_error['yaw']:.3f}, "
                    #f"Torques -> Roll: {cmd.angular.x:.2f}, Pitch: {cmd.angular.y:.2f}, Yaw: {cmd.angular.z:.2f}")


if __name__ == "__main__":
    drone_ns = rospy.get_param("~namespace", "D1")
    controller = DronePIDController(namespace=drone_ns)
    rospy.spin()
