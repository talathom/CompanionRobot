# -*- coding: utf-8 -*-
"""
Created on Thu Sep 26 14:38:40 2019

@author: thomas
"""
import sys
import rospy
import cv2
import numpy as np
from geometry_msgs.msg import Twist
from aruco_detector.msg import ArucoMessage
from face_recognition.msg import FaceMessage
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import time

PI = 3.1415926535897
canMove = True
TWISTMSG = Twist()
SPEED = 0.15
OLD_IMG = ""
firstRun = True
LEFT_BOUND = 215
RIGHT_BOUND = 400
AR_ID = 0

class image_convertor:
    def __init__(self):
        self.bridge = CvBridge()
        self.rate = rospy.Rate(10)
        #Subscribe to Image Topic, uses the camera to see colors in front of the robot
        self.aruco_sub = rospy.Subscriber("aruco_detector/ArucoMessage", ArucoMessage, self.aruco_callback)
        #Subscribe to face topic to respond to faces
        self.face_sub = rospy.Subscriber("face_recognizer/FaceMessage", FaceMessage, self.face_callback)
        self.vision_sub = rospy.Subscriber("camera/rgb/image_raw", Image, self.image_callback)
        #self.image_sub = rospy.Subscriber("/camera/rgb/image_color", Image, self.image_callback)
        #Creates a publisher for our base so that we can move the robot by publishing Twist Messages
        self.pub = rospy.Publisher("/mobile_base/commands/velocity", Twist, queue_size=10)
        rospy.spin()
        
    def image_callback(self, data):
        try:
          cv_image = np.asarray(self.bridge.imgmsg_to_cv2(data, 'bgr8'))
          cv2.imshow("Vision", cv_image)
          if cv2.waitKey(1) == 27:
              cv2.destroyAllWindows() # Close windows
        except CvBridgeError as e:
            print(e)
    
    def aruco_callback(self, data):
        print(str(data.x))
        global LEFT_BOUND
        global RIGHT_BOUND
        global canMove
        
        x = data.x
        y = data.y
        z = data.z
        current_id = data.id
        print(str(x) +" "+ str(z))
        print(str(AR_ID) +" = ID")

        if AR_ID == current_id:
            canMove = True
            if z > 1:
                if x < LEFT_BOUND:
                    self.turn(False)
                elif x > RIGHT_BOUND:
                    self.turn(True)
                else:
                    self.moveForward()
            else:
                self.stop()
        elif current_id == 1:
            canMove = False
        elif current_id == 2:
            if z > 1 and canMove:
                self.backup()
                
    def face_callback(self, data):
        global LEFT_BOUND
        global RIGHT_BOUND
        global canMove
        
        MIN_SAFE_DISTANCE = 2.3
        
        x = data.x
        y = data.y
        z = data.z
        name = data.name
        if name == "Thomas":
            print("FACE at "+ str(z) +" feet away")
            if z > MIN_SAFE_DISTANCE and canMove:
                if x < LEFT_BOUND:
                    self.turn(False)
                elif x > RIGHT_BOUND:
                    self.turn(True)
                else:
                    self.moveForward()
            else:
                self.stop()
            
    def moveForward(self): 
        global SPEED
        #if statements to determine distance from moving color
        TWISTMSG.linear.x = SPEED
        TWISTMSG.angular.z = 0
        self.pub.publish(TWISTMSG)

    def turn(self, Clockwise):
        global SPEED
        SPEED = .15
        #If we want to move clockwise our velocity must be negative, as a result we also have to negat the endTime result as we cannot have negative time
        if Clockwise:
            TWISTMSG.angular.z = -SPEED*5 #Set the angular speed to the speed we specified
        else:
            print("Left Turn")
            TWISTMSG.angular.z = SPEED*5 #Set the angular speed to the speed we specified
        self.pub.publish(TWISTMSG)

    def speedUp(self):
        #speed up the robot by setting the speed higher
        global SPEED
        SPEED = .45


    def defaultSpeed(self):
        #sets the robot speed back to the default speeed
        global SPEED
        SPEED = .15
    
    def stop(self):
        #stops the robot by publishing a message with V = 0 in all fields
        
        TWISTMSG.linear.x = 0
        TWISTMSG.linear.y = 0
        TWISTMSG.linear.z = 0
        TWISTMSG.angular.x = 0
        TWISTMSG.angular.y = 0
        TWISTMSG.angular.z = 0
        self.pub.publish(TWISTMSG)
    
    def slowDown(self):
        #slows down the robot gradually
        global SPEED
        SPEED -= .02
        if SPEED < 0:
            SPEED = 0
        self.pub.publish(TWISTMSG)
        
    def backup(self):
        global SPEED
        TWISTMSG.linear.x = -SPEED
        TWISTMSG.linear.y = 0
        TWISTMSG.linear.z = 0
        TWISTMSG.angular.x = 0
        TWISTMSG.angular.y = 0
        TWISTMSG.angular.z = 0
        self.pub.publish(TWISTMSG)
        
# Main function creates an instance of our image view class
def main(args):
    print("Main running")
    rospy.init_node('image_convertor', anonymous=True)
    ic = image_convertor()
    try:
    	rospy.spin()
    except KeyboardInterrupt:
    	print("Shutting down")
    cv2.destroyAllWindows()
        

    
# Main calls our main function
if __name__ == '__main__':   
    main(sys.argv)
        

        
