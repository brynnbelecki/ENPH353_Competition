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
        self.display_interval = rospy.Duration(0.02)  

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
                x, y = map(int, pt)
                cv2.circle(cv_image, (x, y), 5, (0, 0, 255), -1)

        if rect_corners is None:
            # No rectangle → publish invalid coords
            coords_msg = Float32MultiArray()
            coords_msg.data = [-1, -1, -1, -1]
            self.pub.publish(coords_msg)
            return

        cyan_center = self.find_color_center(cv_image, lower=np.array([200, 200, 0]), upper=np.array([255, 255, 50]))
        yellow_center = self.find_color_center(cv_image, lower=np.array([0, 200, 200]), upper=np.array([50, 255, 255]))

        if cyan_center is not None:
            cv2.circle(cv_image, (int(cyan_center[0]),int(cyan_center[1])), 2, (255, 255, 0), -1)
        if yellow_center is not None:
            cv2.circle(cv_image, (int(yellow_center[0]),int(yellow_center[1])), 2, (0, 255, 255), -1)

        
        coords_msg = Float32MultiArray()

        if cyan_center is not None:
            (x_c, y_c), = self.get_relative_coords([cyan_center], rect_corners)
        else:
            x_c, y_c = -1, -1

        if yellow_center is not None:
            (x_y, y_y), = self.get_relative_coords([yellow_center], rect_corners)
        else:
            x_y, y_y = -1, -1

        coords_msg = Float32MultiArray()
        coords_msg.data = [x_c, y_c, x_y, y_y]

        self.pub.publish(coords_msg)

    

    def find_rectangle_corners(self, img):
 

        lower = np.array([175, 175, 175], dtype=np.uint8)
        upper = np.array([181, 181, 181], dtype=np.uint8)
        mask = cv2.inRange(img, lower, upper)  
        mask = cv2.bitwise_not(mask)     

        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.erode(mask, kernel, iterations=1)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None

        cnt = max(contours, key=cv2.contourArea)

        epsilon = 0.02 * cv2.arcLength(cnt, True)
        approx = cv2.approxPolyDP(cnt, epsilon, True)

        if len(approx) != 4:
            x, y, w, h = cv2.boundingRect(cnt)
            approx = np.array([
                [x, y],
                [x + w, y],
                [x + w, y + h],
                [x, y + h]
            ], dtype=np.float32)

        pts = approx.reshape(4, 2).astype(np.float32)

        s = pts.sum(axis=1)
        diff = np.diff(pts, axis=1)
        tl = pts[np.argmin(s)]
        br = pts[np.argmax(s)]
        tr = pts[np.argmin(diff)]
        bl = pts[np.argmax(diff)]

        return np.array([tl, tr, br, bl], dtype=np.float32)



    def find_color_center(self, img, lower, upper):
        mask = cv2.inRange(img, lower, upper)
        moments = cv2.moments(mask)
        if moments["m00"] == 0:
            return None
        cx = (moments["m10"] / moments["m00"])
        cy = (moments["m01"] / moments["m00"])
        return [cx, cy]
    
    def get_relative_coords(self, points, corners):

        dst = np.array([
            [0, 0],   
            [1, 0],   
            [1, 1],   
            [0, 1]    
        ], dtype=np.float32)

        H, _ = cv2.findHomography(corners, dst)

        results = []

        for point in points:
            px = np.array([point[0], point[1], 1.0])
            uvw = H @ px
            u = float(uvw[0] / uvw[2])
            v = float(uvw[1] / uvw[2])
            results.append((u, v))

        return results


if __name__ == "__main__":
    node = DroneVisionNode()
    rospy.spin()
    cv2.destroyAllWindows()
