#! /usr/bin/env python3

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

class Car:

    def __init__(self):
        rospy.init_node('topicPublisher')

        # initial position from robot.launch file
        # -x 5.5 -y 2.5 -z 0.2 -R 0.0 -P 0.0 -Y -1.57 
        self.respawn([5.5, 2.5, 0.2, 0.0, 0.0, -0.7071, 0.7071])

        rate = rospy.Rate(30)
        self.last_processed_time = time.time()
        self.min_interval = 0.1
        self.mostRecentCV = None
        self.interface = Interface(self)
        
        #initImage = False

        while not rospy.is_shutdown():
            self.subClock = rospy.Subscriber('/clock', Clock)
            self.pubSpeed = rospy.Publisher('/B1/cmd_vel', Twist, queue_size=1)
            self.pubScore = rospy.Publisher('/score_tracker', String, queue_size=10)
            self.subCamera = rospy.Subscriber("/B1/rrbot/camera1/image_raw", Image, self.process)
            self.interface.image()
            #self.interface.show()

            #self.subCamera = rospy.Subscriber("/B1/rrbot/camera1/image_raw", Image, self.interface)

            rate.sleep()  
    
    #self.move (Twist object)
    def __getattribute__(self, name):
        return super().__getattribute__(name)

    ## respawn robot to starting position (illegal teleportation)
    def respawn(self, position):

        msg = ModelState()
        msg.model_name = 'B1'

        msg.pose.position.x = position[0]
        msg.pose.position.y = position[1]
        msg.pose.position.z = position[2]
        msg.pose.orientation.x = position[3]
        msg.pose.orientation.y = position[4]
        msg.pose.orientation.z = position[5]
        msg.pose.orientation.w = position[6]

        rospy.wait_for_service('/gazebo/set_model_state')
        try:
            set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            resp = set_state( msg )

        except rospy.ServiceException:
            print ("Service call failed")
 
    ## Image processing controller (main control loop)
    #
    #< @param image input image from camera subscriber node
    def process(self, image):
    
        current_time = time.time()
        if current_time - self.last_processed_time >= self.min_interval:
        #convert image to cv encoding and display
            cvBridge = CvBridge()
            try:
                cvImage = cvBridge.imgmsg_to_cv2(image, desired_encoding="bgr8")
            except CvBridgeError as e:
                rospy.logerr(e)
            self.mostRecentCV = cvImage
            
            self.drive(cvImage)
           
            self.last_processed_time = current_time
        
    ## Line following algorithm
    #
    #< @param cvImage robot camera image
    def drive(self, cvImage):
         #create and publish motion to robot
        self.move = Twist()

        gray = cv.cvtColor(cvImage, cv.COLOR_BGR2GRAY)
        
        THRESHOLD = 100 #From looking at printed frame
        _, threshold = cv.threshold(gray, THRESHOLD, 255, cv.THRESH_BINARY)
        height, width = threshold.shape

        error = 0
        kp = 0.006 #optimal kp is 0.005 at speed of 0.5 (linear.x)
        kd = 0.01

        left1 = width / 2
        right1 = width / 2
        left2 = width / 2
        right2 = width / 2
        found = False
        y = height - 100
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

        # print("size")
        # print(cvImage.shape)

        print("left1")
        print(left1)
        print("right1")
        print(right1)

        print("left2")
        print(left2)
        print("right2")
        print(right2)

        lineLoc = (right1 + left2) / 2.0 #middle of line
        #print(left2 - right1)
        #print("line")
        # print(lineLoc)
        #error is deviation from middle of screen 
        error = width / 2.0 - lineLoc
        #print(error)
        
        proportional = -1 * kp * error 
        if error == width / 2.0: 
            proportional *= -1
        if abs(error) > 300:
            proportional *= 3
        self.move.linear.x = .7
        #print(proportional)
        self.move.angular.z = proportional

        #cv.imshow("image", cvImage)
        cv.waitKey(2)

        self.pubSpeed.publish(self.move)

class Interface(QtWidgets.QMainWindow):
    
    def __init__(self, car):
        super(Interface, self).__init__()

        self.car = car
        uic.loadUi("/home/fizzer/ros_ws/src/car_controller/src/nodes/353Competition.ui", self) 

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_image)
        self.timer.start(50)

        test_pix = QtGui.QPixmap(400, 300)
        test_pix.fill(QtGui.QColor("green"))
        self.Camera.setPixmap(test_pix)
        print("Set TEST pixmap")

        layout = QtWidgets.QVBoxLayout(self.centralwidget)
        self.centralwidget.setLayout(layout)

        layout.addWidget(self.Camera)
        layout.addWidget(self.Twist)


        #self.timer = QTimer()
        #self.timer.timeout.connect(self.image)
        #self.timer.start(30)

        # try:
        #     self.image()
        # except AttributeError:
        #     return
        # self.show()

    def image(self):
        print(self.__dict__.keys())
        self.Camera.setScaledContents(True)
        self.Camera.setStyleSheet("background-color: red;")
        self.Camera.raise_()
        print("Camera:", self.Camera)
        self.show()
        print("Camera size:", self.Camera.width(), self.Camera.height())

        try:
            frame = self.car.mostRecentCV

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

    # def image(self):
        
    #     try: 
    #         image = self.convert_cv_to_pixmap(self.car.mostRecentCV)
    #         self.Camera.setPixmap(image)
    #         self.Camera.repaint() 
    #         print("here")
    #     except Exception as e:
    #         rospy.logerr(e)
    #     #print("here2")
    #     #self.show()

    #     return True

        # self._cam_id = 0
        # self._cam_fps = 2
        # self._is_cam_enabled = False
        # self._is_template_loaded = False

        # self.browse_button.clicked.connect(self.SLOT_browse_button)
        # self.toggle_cam_button.clicked.connect(self.SLOT_toggle_camera)

    ## Converts cv image to pixmap format
    #
    # Source: stackoverflow.com/questions/34232632/
    def convert_cv_to_pixmap(self, cv_img):
        cv_img = cv_img.copy()
        cv_img = cv.cvtColor(cv_img, cv.COLOR_BGR2RGB)
        height, width, channel = cv_img.shape
        bytesPerLine = channel * width
        q_img = QtGui.QImage(cv_img.data, width, height, 
                        bytesPerLine, QtGui.QImage.Format_RGB888)
        return QtGui.QPixmap.fromImage(q_img)

if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    car = Car()
    sys.exit(app.exec_())


