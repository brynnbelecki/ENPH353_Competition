#!/usr/bin/env python3
import rospy
from std_msgs.msg import Bool
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
import subprocess
import os
from geometry_msgs.msg import Pose, Quaternion
from tf.transformations import quaternion_from_euler

class DroneReset:
    def __init__(self):
        rospy.init_node("drone_reset_node")
        self.reset_sub = rospy.Subscriber("/drone_reset", Bool, self.reset_callback)
        rospy.loginfo("[DroneReset] Node started, waiting for reset messages...")

        # Define starting poses for D1 and D2 based on your launch args
        self.starting_poses = {
            "D1": Pose(),
            "D2": Pose()
        }

        # D1: -x 5.5, -y 2.5, -z 0.1, -R 0, -P 0, -Y -1.57
        q1 = quaternion_from_euler(0.0, 0.0, -1.57)
        self.starting_poses["D1"].position.x = 5.5
        self.starting_poses["D1"].position.y = 2.5
        self.starting_poses["D1"].position.z = 0.1
        self.starting_poses["D1"].orientation = Quaternion(*q1)

        # D2: -x 5.5, -y 2.0, -z 0.1, -R 0, -P 0, -Y -1.256
        q2 = quaternion_from_euler(0.0, 0.0, 1.57)
        self.starting_poses["D2"].position.x = 5.5
        self.starting_poses["D2"].position.y = 2.0
        self.starting_poses["D2"].position.z = 0.1
        self.starting_poses["D2"].orientation = Quaternion(*q2)

    def reset_callback(self, msg):
        if not msg.data:
            return
        rospy.loginfo("[DroneReset] Reset triggered!")

        # 1. Teleport drones
        self.teleport_drone("D1", self.starting_poses["D1"])
        self.teleport_drone("D2", self.starting_poses["D2"])

        rospy.sleep(0.5)  # give Gazebo a moment

        # 2. Kill relevant nodes
        nodes_to_kill = [
            "/competition_master_control",
            "/linear_control",
            "/drone_2_control",
            "/drone_2_rotational"
        ]
        for n in nodes_to_kill:
            os.system(f"rosnode kill {n}")

        rospy.sleep(1)

        # 3. Relaunch drones
        subprocess.Popen(["roslaunch", "drone", "drone.launch"])
        subprocess.Popen(["roslaunch", "drone", "drone2.launch"])

        rospy.loginfo("[DroneReset] Drones reset and nodes relaunched!")

    def teleport_drone(self, name, pose):
        rospy.wait_for_service('/gazebo/set_model_state')
        try:
            set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            state_msg = ModelState()
            state_msg.model_name = name
            state_msg.pose = pose
            state_msg.twist.linear.x = 0
            state_msg.twist.linear.y = 0
            state_msg.twist.linear.z = 0
            state_msg.twist.angular.x = 0
            state_msg.twist.angular.y = 0
            state_msg.twist.angular.z = 0
            state_msg.reference_frame = "world"
            set_state(state_msg)
            rospy.loginfo(f"[DroneReset] Teleported {name}")
        except rospy.ServiceException as e:
            rospy.logerr(f"Failed to teleport {name}: {e}")

if __name__ == "__main__":
    try:
        DroneReset()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
