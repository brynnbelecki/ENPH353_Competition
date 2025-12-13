#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32MultiArray, Bool
from std_msgs.msg import String 
import numpy as np

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError 
from ultralytics import YOLO

class DroneTargetTrigger:
    def __init__(self):
        rospy.init_node("drone_target_trigger")
        self.pub = rospy.Publisher("/drone_target", Float32MultiArray, queue_size=10)
        rospy.Subscriber("/drone_coords", Float32MultiArray, self.coords_callback)
        self.reset_sub = rospy.Subscriber("/drone_reset", Bool, self.reset_callback)
        self.reset = False

        camera_path = "/D2/rgb_camera/image_raw"
        rospy.Subscriber(camera_path, Image, self.process) 
        self.pubScore = rospy.Publisher('/score_tracker', String, queue_size=10)
        self.modelSign = YOLO("/home/fizzer/ENPH353_Competition/src/drone/YOLO/FindSign.pt")
        self.modelLetters = YOLO("/home/fizzer/ENPH353_Competition/src/drone/YOLO/20251203.pt")
        

        # starts timer
        rospy.sleep(5)
        for _ in range(10):
            self.pubScore.publish(f"Team 7,password,0,NA")

        self.targets = [
            [0.045, 0.863, 0.5, -0.8/2],   # sign 1          
            [0.056, 0.32, 0.5, -1.1/2],    # sign 2
            [0.125, 0.2, 0.5, 1],        # sign 3
            [0.45, 0.35, 0.5, 0.8/2],      # sign 4
            [0.45, 0.85, 0.5, -0.9/2],     # sign 5  
            [0.74, 0.76, 0.5, 0.8],        # sign 6  
            [0.87, 0.12, 0.5, 0],       # sign 7  
            [0.85, 0.12, 3, 0],      # waypoint (special: no Z=0 drop, just wait)
            [0.63, 0.27, 3, 0], # waypoint (special: no Z=0 drop, just wait)
            [0.645, 0.275, 0.5, 0],       # final sign
            [0.75, 0.1, 2.5, 0], #slam into the tunnel
            [0.75, 0.1, 0, 0]
        ]

        self.clueboard_message = {"SIZE": "", "VICTIM": "", "CRIME": "", "TIME": "", "PLACE": "", "MOTIVE": "", "WEAPON": "", "BANDIT": ""}
        self.clueboard_count = {"SIZE": 0, "VICTIM": 0, "CRIME": 0, "TIME": 0, "PLACE": 0, "MOTIVE": 0, "WEAPON": 0, "BANDIT": 0}
        self.clue_send_count = 0
        self.clue_id = 1

        self.xy_threshold = 0.0075
        self.threshold_count_required = 25

        self.curr_x = 0.0
        self.curr_y = 0.0
        self.curr_z = 0.0

        self.rate = rospy.Rate(50)
        self.main_loop()
    
    def reset_callback(self, msg):

            self.reset = msg.data
            print(self.reset)
            self.pubScore.publish(f"Team 7,password,-1,NA")

            if self.reset:
                rospy.logwarn("Reset flag received! Resetting robot state...")

                self.clue_send_count = 0
                self.clue_id = 1
                self.clueboard_message = ""
                self.clueboard_count = 0
                self.curr_x = 0.0
                self.curr_y = 0.0
                self.curr_z = 0.0

                self.reset = True


   

    def coords_callback(self, msg: Float32MultiArray):
        if len(msg.data) < 4:
            return

        self.curr_x = (msg.data[0] + msg.data[2]) / 2
        self.curr_y = (msg.data[1] + msg.data[3]) / 2


    def main_loop(self):
        i = 0
        while not rospy.is_shutdown():
            while i < (len(self.targets) -1 ):
                print(i)

                if self.reset:
                    i = 0
                    self.pubScore.publish(f"Team 7,password,-1,NA")
                    rospy.sleep(2)
                    self.reset = False
                    break

                target = self.targets[i]
                tx, ty, tz, yaw = target

                is_waypoint = (tz > 1.5)

                self.publish_target(tx, ty, tz, yaw)
                rospy.loginfo(f"Heading to target XY: [{tx:.3f}, {ty:.3f}] Z={tz}")

                consecutive_count = 0

                if is_waypoint:
                    rospy.sleep(1)
                    i += 1
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

                # publish sign once read -- should wait until this is successful before continuing
                notSent = True
                while notSent: #this is non-blocking (pretty sure bc subscriber should be its own thread)
                    if  list(self.clueboard_count.values())[self.clue_send_count] >= 5 and list(self.clueboard_message.values())[self.clue_send_count] is not "":
                        self.pubScore.publish(f"Team 7,password,{self.clue_id},{list(self.clueboard_message.values())[self.clue_send_count]}")
                        
                        self.clue_id += 1
                        self.clue_send_count += 1
                        notSent = False
                    elif self.clue_id > 8:
                        notSent = False 
                    else:
                        rospy.sleep(0.01)

                print(f"target: {target}")
                print(f"signs read: {self.clueboard_message}") 
                print(f"count: {self.clueboard_count}") 

                rospy.sleep(0.5)

                self.publish_target(tx, ty, tz, yaw)
                rospy.sleep(0.25)
                i +=1

            # stops timer -- should not reach here unless we have had a successful run in which case we don't need to reset and should stop

            self.pubScore.publish(f"Team 7,password,-1,NA")
            self.publish_target(tx, ty, 0, yaw)
            return

    def publish_target(self, x, y, z, yaw):
        msg = Float32MultiArray()
        msg.data = [x, y, z, yaw * np.pi]
        self.pub.publish(msg)

    ## Processes input images
    def process(self, image):
        cvBridge = CvBridge()
        try:
            cvImage = cvBridge.imgmsg_to_cv2(image, desired_encoding="bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)

        #look for signs every frame        
        self.YOLO(cvImage)  
        
                
    ## runs sign YOLO model on image to extract sign and then on sign to extract individual letters
    #
    #< @param cvImage the input image
    def YOLO(self, cvImage):
        # Trained YOLO models


        # Only include if model > 80% confident    
        results = self.modelSign(source=cvImage, conf = 0.85, show=False)

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
                cropped_image_np = cvImage[y1 + 25:y2-25, x1+25:x2-25]

                message = self.modelLetters(source=cropped_image_np, conf=0.5, save=False, project = "/home/fizzer/ros_ws/src/car_controller/neural/Read20251129")
                self.getClue(self.modelLetters, message)

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
        #print(chars)

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
        #print(row1[label_index])
        #print(row1chars[label_index])
        #print(row2[clue_index])
        #print(row2chars[clue_index])

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

    
if __name__ == "__main__":
    try:
        DroneTargetTrigger()
    except rospy.ROSInterruptException:
        pass
