#! /usr/bin/env python3

import sys
import cv2 as cv
from PyQt5 import QtWidgets, QtGui, QtCore, uic
import numpy as np

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from std_msgs.msg import String  # To publish to /score_tracker
from rosgraph_msgs.msg import Clock

from gazebo_msgs.msg import ModelState #illegal for actual competition
from gazebo_msgs.srv import SetModelState

import cv2 as cv
from cv_bridge import CvBridge, CvBridgeError
import time

from PyQt5 import QtCore, QtGui, QtWidgets, uic
from PyQt5.QtCore import QTimer
from python_qt_binding import loadUi
import sys

class Interface(QtWidgets.QMainWindow):
    def __init__(self,):
        super(Interface, self).__init__()

        # Load interface
        uic.loadUi("/home/fizzer/ros_ws/src/car_controller/src/nodes/353Competition.ui", self)

        self.frame = None

        # Make QLabel scale pixmaps to fit
        self.Camera.setScaledContents(True)

        # Start a timer to simulate live camera updates
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_image)
        self.timer.start(50)  # update every 50 ms (~20 FPS)

        # Create a dummy frame
        self.frame = np.zeros((800, 800, 3), dtype=np.uint8)
        cv.putText(self.frame, "Test Frame", (50, 400), cv.FONT_HERSHEY_SIMPLEX, 3, (0, 255, 0), 5)

    def update_image(self):
        rospy.init_node('topicPublisher')
        self.subCamera = rospy.Subscriber("/B1/rrbot/camera1/image_raw", Image, self.process)

        try:
            frame = self.frame

            if frame is None:
                print("No frame available")
                return
        
            print("Frame shape:", frame.shape)

            pixmap = self.convert_cv_to_pixmap(frame)

            if pixmap is None:
                print("Pixmap conversion failed")
                return

            self.Camera.setScaledContents(True)
            self.Camera.setPixmap(pixmap)
            self.Camera.repaint()   # optional, forces UI update
            self.show()
            print("Image updated")

        except Exception as e:
            print("Image update error:", e)

    def process(self, image):
        cvBridge = CvBridge()
        try:
            cvImage = cvBridge.imgmsg_to_cv2(image, desired_encoding="bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)

        height, width, __ = cvImage.shape
        line_height = height - 300
        start_point = (0, line_height)
        end_point = (width - 1, line_height)
        line_color = (255,0,0)
        line_thickness = 2
        cv.line(cvImage, start_point, end_point, line_color, line_thickness)

        self.frame = cvImage
    
    def convert_cv_to_pixmap(self, cv_img):
        # Ensure image remains the same in memory
        cv_img = cv_img.copy()
        # Convert BGR to RGB
        cv_img = cv.cvtColor(cv_img, cv.COLOR_BGR2RGB)
        h, w, ch = cv_img.shape
        bytes_per_line = ch * w
        q_img = QtGui.QImage(cv_img.data, w, h, bytes_per_line, QtGui.QImage.Format_RGB888)
        return QtGui.QPixmap.fromImage(q_img)


if __name__ == "__main__":
    print("init")
    app = QtWidgets.QApplication(sys.argv)
    interface = Interface()
    interface.show()
    sys.exit(app.exec_())