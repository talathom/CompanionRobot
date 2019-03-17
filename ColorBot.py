#!/usr/bin/env python
# Written by Robotics Team 2 Fall 2018

import roslib
roslib.load_manifest('opencv_fun')
import sys
import rospy
import cv2
import time
from geometry_msgs.msg import Twist
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import gc
import numpy as np
from ar_track_alvar.msg import AlvarMarker
from ar_track_alvar.msg import AlvarMarkers

PI = 3.1415926535897
canMove = False
TWISTMSG = Twist()
SPEED = 0.2
x = 320
y = 240
radius = 25
marker = 3

class image_convertor:

	def __init__(self):
		self.bridge = CvBridge()
        	self.rate = rospy.Rate(10)
		#Subscribe to AR Marker Topic callback on spotting a marker
		self.image_sub = rospy.Subscriber("/ar_pose_marker", AlvarMarkers, self.imagecallback)
		#Subscribe to Image Topic, uses the camera to see colors in front of the robot
		self.image_sub = rospy.Subscriber("/camera/rgb/image_color", Image, self.colorcallback)
		#Creates a publisher for our base so that we can move the robot by publishing Twist Messages
		self.pub = rospy.Publisher("/mobile_base/commands/velocity", Twist, queue_size=10)
		rospy.spin()

	# This function needs to be rewritten to be of the following logic
	# If COLOR msg.linear.x = .05
	# If NOT COLOR msg.linear.x = 0
	def moveForward(self): 
		global SPEED
    		#if statements to determine distance from moving color
		TWISTMSG.linear.x = SPEED
	    	TWISTMSG.angular.z = 0
		
		self.pub.publish(TWISTMSG)

	def turn(self, Clockwise):
		global SPEED
		SPEED = .3

		#If we want to move clockwise our velocity must be negative, as a result we also have to negat the endTime result as we cannot have negative time
    		if Clockwise:
			print("Right Turn")
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
	    print(SPEED)
	    SPEED -= .02
	    if SPEED < 0:
		SPEED = 0
	
	def backUp(self):
	    #Moves the robot backwards by negating speed and publishing
	    global SPEED
	    TWISTMSG.linear.x = -SPEED
	    TWISTMSG.angular.z = 0
		
	    self.pub.publish(TWISTMSG)
		

    
	"""
	Callback for when an AR Marker is spotted
	The function first determines if the AR Marker spotted is the correct one
	If it is then we determine whether to turn left or right depend
	"""
	
	def imagecallback(self, data):
	    	global marker
		global canMove
		
		if canMove:
			lowerXBound = -.1 #Lower Bound for straight movement
			upperXBound = .1 #Upper Bound for straight Movement

			lowerZBound = .45 #Lower Bound before backing up
			upperZBound = .8 #Upper Bound before speeding up
			stopZBound = .5 #Upper bound before stopping
		
			for i in range(0, len(data.markers)):
				#Gets the ID of the marker
				posX = data.markers[i].pose.pose.position.x
				posZ = data.markers[i].pose.pose.position.z

				#Verify the marker ID
				if data.markers[i].id == marker: 
					print("Positions")
					print(posX)
					print (posZ)
			
					#Adjusts the bounds if the AR Marker is at a very far range
					if posZ > 1:
						lowerXBound = -.3
						upperXBound = .3
			
					#If Blocks that determine whether we turn or speedup based on X and Z positions
					if posZ >= upperZBound:
						print("Speed Up")
						self.speedUp()
					else:
						self.defaultSpeed()
		
					if  posX > lowerXBound and posX < upperXBound and posZ > stopZBound:
						self.moveForward()
				
					if posX > upperXBound:
						self.turn(True)
				
					elif posX < lowerXBound:
						self.turn(False)

					elif posZ <= stopZBound and posZ > lowerZBound:
						self.stop()

					elif posZ <= lowerZBound:
						print("BACKING UP")
						self.backUp()

					else:
						self.defaultSpeed()

				
	"""
	Callback for a new image message that determines if we've seen a full color paper
	"""

	def colorcallback(self, data):
	    global cv_image
	    global radius
	    global canMove
		
	    try:
		cv_image = np.asarray(self.bridge.imgmsg_to_cv(data, "bgr8"))
	    except CvBridgeError as e:
		print(e)

	    (rows,cols,channels) = cv_image.shape
	    if cols > 60 and rows > 60 :
	      	cv2.circle(cv_image, (x,y), radius, 255) # Creates the target cricle

	    cv2.imshow("Image window", cv_image)
	    cv2.waitKey(3)
		
	    colorCode = self.avgOverall(data)
	    
	    if colorCode == 1:
		canMove = False
	    elif colorCode == 2:
		canMove = True
		
		
	
	"""
	Determines if the Average Overall Color is Red or Green
	Red = 1
	Green = 2
	Neither = 0
	"""
	def avgOverall(self, data):
	    global cv_image
	    global radius
	    i = 0
	    bavg = 0
	    gavg = 0
	    ravg = 0
		#Loops over the whole circle
	    for c in range(y - radius, y + radius):
	            for r in range(x - radius, x + radius):
	                bavg += cv_image[c][r][0]
	                gavg += cv_image[c][r][1]
	                ravg += cv_image[c][r][2]
	                i += 1
	    bavg = bavg / i
	    gavg = gavg / i
	    ravg = ravg / i

	    print(bavg)
	    print(gavg)
	    print(ravg)
	    print("--------------------------------------------------------")
		
	    #Check the average against our definitions of the colors
	    if self.isRed(bavg, gavg, ravg):
		return 1
	    elif self.isGreen(bavg, gavg, ravg):
		return 2
	    else:
		return 0

	"""
	Returns if our given RGB code meets our standard for Red
	"""
	def isRed(self, B,G,R):
	    redmin = 135
	    greenmax = 100
	    bluemax = 100
	    #print(R, " > ", redmin, " and ", greenmax, " >= ", G, " and ", bluemax, " >= ", B)
	    if (R >= redmin) and (greenmax >= G) and (bluemax >= B):
		print("STOP!")
	    	return True
	    else:
	        return False
			
    	"""
    	Returns if our given RGB code meets our standard for Green
    	"""
	def isGreen(self, B,G,R):
	    greenmin = 130
	    redmax = 130
	    bluemax = 130
	    if (G >= greenmin) and (redmax >= R) and (bluemax >= B):
		print("GO!")
		return True
	    else:
	        return False


# Main function creates an instance of our image convertor class
def main(args):
    print("Main running")
    rospy.init_node('image_converter', anonymous=True)
    ic = image_convertor()
    try:
    	rospy.spin()
    except KeyboardInterrupt:
    	print("Shutting down")
    cv2.destroyAllWindows()
        

    
# Main calls our main function
if __name__ == '__main__':   
    main(sys.argv)
    
    
    
