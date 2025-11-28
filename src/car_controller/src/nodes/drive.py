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

from PyQt5 import QtCore, QtGui, QtWidgets
from python_qt_binding import loadUi

import numpy as np

POSITION1 = [5.5, 2.5, 0.2, 0.0, 0.0, -0.7071, 0.7071]
POSITION2 = [0.53, -0.0079, 0.2, 0.0, 0.0, 0.7071, 0.7071]
POSITION3 = [3.93, -0.42, 0.2, 0.0, 0.0, 1, 0]
POSITION4 = [3.93, -2.27, 0.2, 0.0, 0.0, 0, 1]

class Car:

    def __init__(self):
        rospy.init_node('topicPublisher')

        # initial position from robot.launch file
        # -x 5.5 -y 2.5 -z 0.2 -R 0.0 -P 0.0 -Y -1.57 
        self.respawn(POSITION2)

        rate = rospy.Rate(30)
        self.last_processed_time = time.time()
        self.min_interval = 0.2
        
        self.state = "drive, gray road, before ped"
        self.lastError = None
        self.start = True
        count = 0
        self.time = Clock()

        while not rospy.is_shutdown():
            
            self.subCamera = rospy.Subscriber("/B1/rrbot/camera1/image_raw", Image, self.process)
            self.subClock = rospy.Subscriber('/clock', Clock, self.getTime)
            self.pubSpeed = rospy.Publisher('/B1/cmd_vel', Twist, queue_size=1)
            self.pubScore = rospy.Publisher('/score_tracker', String, queue_size=10)

            if self.start:
                self.pubScore.publish(f"Team 7,password,0,NA")
                count += 1
                if count == 10:
                    self.start = False
                #self.start = False
                #print("start")
            
            if self.time.clock.to_sec()  > 240:
                self.pubScore.publish("Team 7,password,-1,NA")

            rate.sleep()

    def getTime(self, time):
        self.time = time

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
        

    def process(self, image):
        global last_processed_time
        current_time = time.time()
        if current_time - self.last_processed_time >= self.min_interval:
            
            #convert image to cv encoding and display
            cvBridge = CvBridge()
            try:
                cvImage = cvBridge.imgmsg_to_cv2(image, desired_encoding="bgr8")
            except CvBridgeError as e:
                rospy.logerr(e)
            
            self.drive(cvImage)
            
            self.last_processed_time = current_time

    ## Line following algorithm
    #
    #< @param cvImage robot camera image
    def drive(self, cvImage):
        #create and publish motion to robot
        move = Twist()

        #gray = cv.cvtColor(cvImage, cv.COLOR_BGR2GRAY)
        #blur = cv.GaussianBlur(gray, (9,9), 0)
        #blur = cv.bilateralFilter(blur,50,40,75)
        # blur1 = cv.blur(cvImage, (10,10))
        # blur2 = cv.blur(blur1, (10,10))
        b, g, r = cv.split(cvImage)

        HEIGHT = 250
        THRESHOLD = 165 #From looking at printed frame
        THRESHOLD_B = 125
        _, threshold = cv.threshold(b, THRESHOLD_B, 255, cv.THRESH_BINARY)
        height, width = threshold.shape

        #remove blobs from thresholded line image to extract line
        nb_blobs, im_with_separated_blobs, stats, _ = cv.connectedComponentsWithStats(threshold)
        sizes = stats[:, cv.CC_STAT_AREA]
        width_s = stats[:, cv.CC_STAT_WIDTH]
        height_s = stats[:, cv.CC_STAT_HEIGHT]
        
        min_size = 2500 
        #min_aspect_ratio = 5
        #or (height_s[index_blob] / width_s[index_blob]) > min_aspect_ratio

        im_result = np.zeros_like(im_with_separated_blobs)
        im_result = im_result.astype(np.uint8)

        # 

        for index_blob in range(1, nb_blobs):
            # print(f"{abs((height_s[index_blob] / width_s[index_blob])) - 1}")
            if sizes[index_blob] >= min_size and not abs((height_s[index_blob] / width_s[index_blob]) - 1) < 0.5:
                # print(f"height: {height_s[index_blob]}")
                # print(f"width: {width_s[index_blob]}")
                im_result[im_with_separated_blobs == index_blob] = 255
        threshold = im_result
        #print("loop")

        error = 0
        kp = 0.023 #0.005 for threshold, 0.015 for blue road, 
        kd = 0.005

        left1 = width / 2
        right1 = width / 2
        left2 = width / 2
        right2 = width / 2
        found = False
        y = height - HEIGHT #250 for 800x800 pixel camera 
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
        
        
        proportional = -1 * kp * error 
        if error == width / 2.0: 
            proportional *= -1
        if abs(error) > 300: #300
            proportional *= 3
        move.linear.x = 0.1 #0.5
        
        if self.lastError == None:
            self.lastError = error
        derivative = -1 * kd * (error - self.lastError)
        self.lastError = error

        #print(proportional)
        move.angular.z = proportional + derivative
        

        #cv.imshow("image", cvImage)
        #cv.waitKey(3)
        if abs(move.angular.z) < 6:
            self.pubSpeed.publish(move)
        if error != 0:
            print(f"movement: {move.angular.z}")
            print(f"error: {error}")
            print(f"time: {time.time()}")

if __name__ == "__main__":
    car = Car()