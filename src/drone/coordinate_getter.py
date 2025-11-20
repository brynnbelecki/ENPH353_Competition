#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge, CvBridgeError

class DroneVisionNode:
    def __init__(self):
        rospy.init_node('drone_vision_node', anonymous=True)

        self.image_topic = '/D1/rgb_camera/image_raw'
        self.pub_topic = '/drone_coords'

        self.bridge = CvBridge()
        self.sub = rospy.Subscriber(self.image_topic, Image, self.image_callback)
        self.pub = rospy.Publisher(self.pub_topic, Float32MultiArray, queue_size=1)

        self.last_display_time = rospy.Time.now()
        self.display_interval = rospy.Duration(0.02)  # 3 seconds

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            rospy.logerr(f"CvBridge Error: {e}")
            return

        # Step 1: Find rectangle corners
        rect_corners = self.find_rectangle_corners(cv_image)
        if rect_corners is not None:
            for pt in rect_corners:
                cv2.circle(cv_image, tuple(pt), 5, (0, 0, 255), -1)

        cyan_center = self.find_color_center(cv_image, lower=np.array([255, 255, 0]), upper=np.array([255, 255, 0]))
        yellow_center = self.find_color_center(cv_image, lower=np.array([0, 255, 255]), upper=np.array([0, 255, 255]))

        if cyan_center is not None:
            cv2.circle(cv_image, tuple(cyan_center), 5, (255, 255, 0), -1)
        if yellow_center is not None:
            cv2.circle(cv_image, tuple(yellow_center), 5, (0, 255, 255), -1)

        coords_msg = Float32MultiArray()
        if rect_corners is not None:
            coords_msg.data.extend(rect_corners.flatten())
        if cyan_center is not None:
            coords_msg.data.extend(cyan_center)
        if yellow_center is not None:
            coords_msg.data.extend(yellow_center)
        self.pub.publish(coords_msg)

        if rospy.Time.now() - self.last_display_time > self.display_interval:
            cv2.imshow("Drone Vision", cv_image)
            cv2.waitKey(1)
            self.last_display_time = rospy.Time.now()

    def find_rectangle_corners(self, img):
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        grad_x = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=3)
        grad_y = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=3)
        edges = cv2.magnitude(grad_x, grad_y)
        edges = np.uint8(np.clip(edges, 0, 255))

        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None

        largest_contour = max(contours, key=cv2.contourArea)
        epsilon = 0.02 * cv2.arcLength(largest_contour, True)
        approx = cv2.approxPolyDP(largest_contour, epsilon, True)

        if len(approx) == 4:
            return approx.reshape(4, 2)
        return None

    def find_color_center(self, img, lower, upper):
        mask = cv2.inRange(img, lower, upper)
        moments = cv2.moments(mask)
        if moments["m00"] == 0:
            return None
        cx = int(moments["m10"] / moments["m00"])
        cy = int(moments["m01"] / moments["m00"])
        return [cx, cy]

if __name__ == "__main__":
    node = DroneVisionNode()
    rospy.spin()
    cv2.destroyAllWindows()
