#!/usr/bin/env python

import roslib
roslib.load_manifest('followme')
import sys
import rospy
import cv2
import time
from geometry_msgs.msg import Twist
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
PI = 3.1415926535897
global boolean
TWISTMSG = Twist()
SPEED = 0.1
x = 320
y = 240
radius = 60
n = 0

class image_converter:

	def __init__(self):
		self.bridge = CvBridge()
                self.rate = rospy.Rate(10)
		self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",Image,self.imagecallback)
		self.image_pub = rospy.Publisher("/image_topic_2",Image)
		self.pub = rospy.Publisher("/mobile_base/commands/velocity", Twist, queue_size=10)

	# This function needs to be rewritten to be of the following logic
	# If COLOR msg.linear.x = .05
	# If NOT COLOR msg.linear.x = 0
	def moveForward(self): 
    		#if statements to determine distance from moving color
		TWISTMSG.linear.x = SPEED
		
		self.pub.publish(TWISTMSG)

    
	def moveBackward(self):
    	#Moves the robot backward
    	#if statement for distance being too close to the object its following
    		TWISTMSG.linear.x = -SPEED
    		self.pub.publish(TWISTMSG)

	def turn(self, Clockwise):

		#If we want to move clockwise our velocity must be negative, as a result we also have to negat the endTime result as we cannot have negative time
    		if Clockwise:
			print("Right Turn")
	        	TWISTMSG.angular.z = -SPEED*5 #Set the angular speed to the speed we specified
    		else:
			print("Left Turn")
        		TWISTMSG.angular.z = SPEED*5 #Set the angular speed to the speed we specified
		self.pub.publish(TWISTMSG)
 		
	    	TWISTMSG.angular.z = 0

	def distance(self):
	#Needs a input
	#Evaluate distance from what it is following
	#changing global variable to true or false
		print("NYI")

	def speedUp(self):
	    	#speed up
	    	global SPEED
	    	SPEED = SPEED + .3
    
	def stop(self):
	    #stops
	    TWISTMSG.linear.x = 0
	    TWISTMSG.linear.y = 0
	    TWISTMSG.linear.z = 0
	    TWISTMSG.angular.x = 0
	    TWISTMSG.angular.y = 0
	    TWISTMSG.angular.z = 0
	    self.pub.publish(TWISTMSG)
    
	def slowDown(self):
	    #slows down
	    global SPEED
	    SPEED = SPEED - .1
    
    
	def turnAround(self):
	    #turns around
	    global SPEED
	    while not rospy.is_shutdown():
	    	if (TWISTMSG.angular.z != 0):
			t0 = rospy.Time.now().to_sec() #current time
	  		current_angle = 0
	  		relative_angle = (180 * 2 * PI / 360) *1.15
	   		while(current_angle < relative_angle):  # while less than 180 degrees
				self.pub.publish(TWISTMSG) # publish msg
	       			t1 = rospy.Time.now().to_sec()   
	       			current_angle = (SPEED*(t1-t0))  # new current angle
	   			self.rate.sleep()
	   			#print (current_angle)
	   			#rospy.loginfo(msg) #Log the new velocity   	 
	   			#pub.publish(msg) #Publish the new velocity
	   		TWISTMSG.angular.z = 0   # stop robot
   		 	rospy.sleep(4)
    
    
	def sprint(self):
    	#fast speed
    		global SPEED
    		SPEED = 1

	def evaluateEnv(self):
	    # define enviromnent
	    print("NYI") 

	def isRed(self, B,G,R):
	    redmin = 135
	    greenmax = 100
	    bluemax = 100
	    if (R >= redmin) and (greenmax >= G) and (bluemax >= B):
		return True
	    else:
	        return False

	def isBlue(self, B,G,R):
	    bluemin = 135
	    greenmax = 100
	    redmax = 100
	    if (B >= bluemin) and (greenmax >= G) and (redmax >= R):
		return True
	    else:
	        return False
    
	def isGreen(self, B,G,R):
	    greenmin = 135
	    redmax = 100
	    bluemax = 100
	    if (G >= greenmin) and (redmax >= R) and (bluemax >= B):
		return True
	    else:
	        return False

	def isBlack(self, B,G,R):
	    greenmax = 50
	    redmax = 50
	    bluemax = 50
	    if (greenmax >= G) and (redmax >= R) and (bluemax >= B):
		return True
	    else:
	        return False
    
	def depthTest(self):
    		return True
    		#Needs to eventually return a number, we'll decide in a if whether to move forward or backward
    
	def focusOnFoot(self):
    		#follows foot
	    print("NYI")

	"""
	Callback for when a new image message is received
	The function first determines if the overall center of the image is the correct color
	If the center mass is not correct then it checks the left and right sides of the target area
	If those areas check out then the robot turns to pursue its target
	"""
	def imagecallback(self, data):
	    global cv_image
	    global radius
            global n
	    try:
		cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
	    except CvBridgeError as e:
		print(e)

	    (rows,cols,channels) = cv_image.shape
	    if cols > 40 and rows > 40 :
	      cv2.circle(cv_image, (x,y), radius, 255) # Creates the target cricle

	    cv2.imshow("Image window", cv_image)
	    cv2.waitKey(3)

	    if self.avgOverall(data): # Checks the center
	        self.moveForward()
	    elif self.avgLeft(data): # Checks the left side
		self.turn(False)
		#self.turn(True)
	    elif self.avgRight(data): # Checks the right side
		self.turn(True)
		#self.turn(False)
	    else:
		# print("Stopping")
	        self.stop()
            print "read image" + str(n)
            #self.rate.sleep()
            n = n + 1
	"""
	Checks if the average of the target circle is the correct color
	"""
	def avgOverall(self, data):
	    global cv_image
	    global radius
	    i = 0
	    bavg = 0
	    gavg = 0
	    ravg = 0
	    for c in range(y - radius, y + radius):
                    #if (c % 2 == 0):
                    if (c % 2 == 0 or c % 3 == 0):
                        continue
	            for r in range(x - radius, x + radius):
                        #if (r % 2 == 0):
                        if (r % 2 == 0 or r % 3 == 0):
                            continue  
	                bavg += cv_image[c][r][0]
	                gavg += cv_image[c][r][1]
	                ravg += cv_image[c][r][2]
	                i += 1
	    bavg = bavg / i
	    gavg = gavg / i
	    ravg = ravg / i
            print "blue: " + str(bavg)
            print "\ngreen: " + str(gavg)
            print "\nred: " + str(ravg) + "\n"
	    if self.isBlack(bavg, gavg, ravg):
		return True
	    else:
		return False

	"""
	Checks if the average of the left side of target circle is the correct color	
	"""
	def avgLeft(self, data):
	    global cv_image
	    global radius
	    i = 0
	    bavg = 0
	    gavg = 0
	    ravg = 0
	    for c in range(y - radius, y):
                    if (c % 2 == 0):
                    #if (c % 2 == 0 or c % 3 == 0):
                        continue
	            for r in range(x - radius, x):
                        if (r % 2 == 0):
                        #if (r % 2 == 0 or r % 3 == 0):
                            continue  
	                bavg += cv_image[c][r][0]
	                gavg += cv_image[c][r][1]
	                ravg += cv_image[c][r][2]
	                i += 1
	    bavg = bavg / i
	    gavg = gavg / i
	    ravg = ravg / i
	    if self.isBlack(bavg, gavg, ravg):
		return True
	    else:
		return False

	"""
	Checks if the average of the right side of target circle is the correct color
	"""
	def avgRight(self, data):
	    global cv_image
	    global radius
	    i = 0
	    bavg = 0
	    gavg = 0
	    ravg = 0
	    for c in range(y, y + radius):
                    if (c % 2 == 0):
                    #if (c % 2 == 0 or c % 3 == 0):
                        continue
	            for r in range(x, x + radius):
                        if (r % 2 == 0):
                        #if (r % 2 == 0 or r % 3 == 0):
                            continue  
	                bavg += cv_image[c][r][0]
	                gavg += cv_image[c][r][1]
	                ravg += cv_image[c][r][2]
	                i += 1
	    bavg = bavg / i
	    gavg = gavg / i
	    ravg = ravg / i
	    if self.isBlack(bavg, gavg, ravg):
		return True
	    else:
		return False
    

def main(args):
    global boolean
    boolean = False
    print("Main running")
    rospy.init_node('image_converter', anonymous=True)
    ic = image_converter()
    try:
    	rospy.spin()
    except KeyboardInterrupt:
    	print("Shutting down")
    cv2.destroyAllWindows()
        

    
# Copypasta from Lab 7, this code executes when our main function executes    
if __name__ == '__main__':   
    main(sys.argv)
    #try:  
	#talker()     
    #except rospy.ROSInterruptException: pass    
    
    
    
