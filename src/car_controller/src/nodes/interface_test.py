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

from ultralytics import YOLO

class Interface(QtWidgets.QMainWindow):
    def __init__(self,):
        super(Interface, self).__init__()

        # Load interface
        uic.loadUi("/home/fizzer/ros_ws/src/car_controller/src/nodes/353Competition.ui", self)

        self.frame = None
        self.imNum = 0

        # Make QLabel scale pixmaps to fit
        self.Camera.setScaledContents(True)

        # Start a timer to simulate live camera updates
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_image)
        self.timer.start(100)  # update every 100 ms (~10 FPS)

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
        
            #print("Frame shape:", frame.shape)

            pixmap = self.convert_cv_to_pixmap(frame)

            if pixmap is None:
                print("Pixmap conversion failed")
                return

            self.Camera.setScaledContents(True)
            self.Camera.setPixmap(pixmap)
            self.Camera.repaint()   # optional, forces UI update
            self.show()
            #print("Image updated")

        except Exception as e:
            print("Image update error:", e)

    def process(self, image):
        cvBridge = CvBridge()
        try:
            cvImage = cvBridge.imgmsg_to_cv2(image, desired_encoding="bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)

        #look for signs every 50 frames
        if self.imNum % 20 == 0:
            #print frames to Pictures folder (used to train YOLO)
            #cv.imwrite(f'/home/fizzer/ros_ws/src/car_controller/neural/Pictures/drive{self.imNum}.png', self.frame)
            
            self.YOLO(cvImage)
        
        self.imNum += 1

    def YOLO(self, cvImage):
         ### YOLO
        modelSign = YOLO("/home/fizzer/ros_ws/src/car_controller/neural/YOLO/FindSign.pt")
        modelLetters = YOLO("/home/fizzer/ros_ws/src/car_controller/neural/YOLO/ReadSignLetters.pt")

        # Only include if model > 80% confident    
        results = modelSign(source=cvImage, conf = 0.80, show=False)
        print(len(results))  

        # Get the annotated image as a NumPy array (BGR format)
        if len(results) == 1:
            # Access the first result object (if processing a single image)
            first_result = results[0]
            annotated_image_np = first_result.plot()
            #self.frame = annotated_image_np

            box = first_result.boxes.xyxy
            if box is None or len(box) == 0:
                self.frame = annotated_image_np

            else: 
                #crop sign from image
                #print(f"box: {box}")
                x1, y1, x2, y2 = map(int, box[0])
                cropped_image_np = cvImage[y1:y2, x1:x2]

                message = modelLetters(source=cropped_image_np, conf=0.25, save=True, project = "/home/fizzer/ros_ws/src/car_controller/neural/Read20251117")
                letters = message[0].plot()
                self.frame = letters

    def lineFollow(cvImage):
        gray = cv.cvtColor(cvImage, cv.COLOR_BGR2GRAY)
        b, g, r = cv.split(cvImage)
        
        THRESHOLD = 130 #From looking at printed frame
        _, threshold = cv.threshold(b, THRESHOLD, 255, cv.THRESH_BINARY)

        height, width, __ = cvImage.shape
        line_height = height - 300
        start_point = (0, line_height)
        end_point = (width - 1, line_height)
        line_color = (0,0,255)
        line_thickness = 2
        cv.line(cvImage, start_point, end_point, line_color, line_thickness)
        
        height, width = threshold.shape
        left1 = width / 2
        right1 = width / 2
        left2 = width / 2
        right2 = width / 2
        found = False
        y = height - 250
        for x in range(1, width):
            if not found:
                if threshold[y - 1, x-1] - threshold[y - 1, x] == 1:            
                    left1 = x
                elif threshold[y - 1, x-1] - threshold[y - 1, x] == 255:                    
                    right1 = x
                    found = True
            else:
                if threshold[y - 1, x-1] - threshold[y - 1, x] == 1:            
                    left2 = x
                elif threshold[y - 1, x-1] - threshold[y - 1, x] == 255:                    
                    right2 = x

        height, width, __ = cvImage.shape
        line_width = [int(left1), int(right1), int(left2), int(right2)]
        for width in line_width:
            if width != 400:
                start_point = (width, 0)
                end_point = (width, int(height) - 1)
                line_color = (0,0,255)
                line_thickness = 2
                cv.line(cvImage, start_point, end_point, line_color, line_thickness)

        return cvImage
    
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