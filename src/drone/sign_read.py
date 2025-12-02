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
from std_msgs.msg import Bool

from gazebo_msgs.msg import ModelState #illegal for actual competition
from gazebo_msgs.srv import SetModelState

import cv2 as cv
from cv_bridge import CvBridge, CvBridgeError 
import time

from PyQt5 import QtCore, QtGui, QtWidgets, uic
from PyQt5.QtCore import QTimer
from python_qt_binding import loadUi

from ultralytics import YOLO

class ReadSign():
    def __init__(self, cameraPath):

        rospy.init_node(f'sign_read')
        self.imNum = 0

        self.clueboard_message = {"SIZE": "", "VICTIM": "", "CRIME": "", "TIME": "", "PLACE": "", "MOTIVE": "", "WEAPON": "", "BANDIT": ""}
        self.clueboard_count = {"SIZE": 0, "VICTIM": 0, "CRIME": 0, "TIME": 0, "PLACE": 0, "MOTIVE": 0, "WEAPON": 0, "BANDIT": 0}

        # Publishers / Subscribers
        self.sub_camera = rospy.Subscriber(cameraPath, Image, self.process) 

        rospy.spin()

    ## Processes input images
    def process(self, image):
        cvBridge = CvBridge()
        try:
            cvImage = cvBridge.imgmsg_to_cv2(image, desired_encoding="bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)

        #look for signs every 10 frames
        if self.imNum % 10 == 0:         
                self.YOLO(cvImage)  
        self.imNum += 1
                
    ## runs sign YOLO model on image to extract sign and then on sign to extract individual letters
    #
    #< @param cvImage the input image
    def YOLO(self, cvImage):
        # Trained YOLO models
        modelSign = YOLO("/home/fizzer/ros_ws/src/car_controller/neural/YOLO/FindSign.pt")
        modelLetters = YOLO("/home/fizzer/ENPH353_Competition/src/drone/YOLO/20251201.pt")

        # Only include if model > 80% confident    
        results = modelSign(source=cvImage, conf = 0.80, show=False)

        # Get the annotated image as a NumPy array (BGR format)
        if len(results) == 1:
            # Access the first result
            first_result = results[0]
            annotated_image_np = first_result.plot()

            box = first_result.boxes.xyxy
            if box is None or len(box) == 0:
                return

            else: 
                #crop sign from image
                x1, y1, x2, y2 = map(int, box[0])
                cropped_image_np = cvImage[y1:y2, x1:x2]

                message = modelLetters(source=cropped_image_np, conf=0.25, save=False, project = "/home/fizzer/ros_ws/src/car_controller/neural/Read20251129")
                letters = message[0].plot()
                #print(f"message: {message[0]}")
                #self.frame = annotated_image_np
                self.getClue(modelLetters, message)

    ## Determines topic and clue from YOLO output
    #
    #< @param model the YOLO model used to extract clueboard letters
    #< @param message the raw YOLO output from running the model
    def getClue(self, model, message):
        #extract model results and locations on sign image 
        topx = []
        topy = []
        chars = []
        
        boxes = message[0].boxes
        xyxy = boxes.xyxy
        class_ids = boxes.cls
    
        for box, cls in zip(xyxy, class_ids):
            region = box.tolist()       # [x1, y1, x2, y2]
            topx.append(region[0])
            topy.append(region[1])
            class_id = int(cls)         # integer class ID (match to YOLO classes)
            class_name = model.names[class_id]
            chars.append(str(class_name))
        print(chars)

        #categorize by vertical position on image
        # based on model certainty, so one of row1 and row2 will be the topic and the other will be the clue
        y1 = topy[0]
        row1 = [topx[0]]
        row1chars = [chars[0]]
        row2 = []
        row2chars = []
        for i in range(1,len(topy)): 
            if abs(topy[i] - y1) < 10:
                row1.append(topx[i])
                row1chars.append(chars[i])
            else:
                row2.append(topx[i])
                row2chars.append(chars[i])

        row1 = np.array(row1)
        row2 = np.array(row2)
        row1chars = np.array(row1chars)
        row2chars = np.array(row2chars)

        # order topic / clue by location on sign
        label_index = np.argsort(row1)
        clue_index = np.argsort(row2)
        print(row1[label_index])
        print(row1chars[label_index])
        print(row2[clue_index])
        print(row2chars[clue_index])

        # check that topic matches available topics and match clue
        self.matchClue(''.join(row1chars[label_index]), ''.join(row2chars[clue_index]))

    ## matches clue to topic
    #
    #< @param topic string, either the clue or the topic
    #< @param clue string, either the clue or the topic
    def matchClue(self, topic, clue):
        for key in list(self.clueboard_message.keys()):
            if topic == key and self.clueboard_count[key] <= 5:
                if clue == self.clueboard_message[key]:
                    self.clueboard_count[key] += 1
                self.clueboard_message[key] = clue
            else:
                if clue == key and self.clueboard_count[key] <= 5:
                    if topic == self.clueboard_message[key]:
                        self.clueboard_count[key] += 1
                    self.clueboard_message[key] = topic

    ## returns True if the model has read the same message 5 times 
    #
    #< @param topic the clueboard topic
    #< @return true if the model has read the same message 5 times, false otherwise
    def clueboard_status(self, topic):
        if self.clueboard_count(topic) == 5: 
            return True
        else:
            return False 

if __name__ == "__main__":
    camera_path = "/B1/rrbot/camera2/image_raw"
    read_sign = ReadSign(camera_path)
    