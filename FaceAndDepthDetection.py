# -*- coding: utf-8 -*-
"""
Created on Wed Sep 18 22:21:56 2019
@author: Thomas
"""

import cv2
from cv2 import aruco

def show_webcam(mirror=False):
    cam = cv2.VideoCapture(0) # Capture Webcam
    BLUE = (255, 0, 0)
    KNOWN_WIDTH_INCHES = 4.55 # AR Marker width in inches
    KNOWN_WIDTH_MM = KNOWN_WIDTH_INCHES * 25.4
    WIDTH = 480 # Image width in pixels
    HEIGHT = 640 # image height in pixels
    PIXEL_SIZE = 310 # Size of AR Marker in Pixels
    DISTANCE0 = 12 # 1 foot in inches
    FOCAL_LENGTH = DISTANCE0 * PIXEL_SIZE / KNOWN_WIDTH_INCHES
    face_cascade = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')
    font = cv2.FONT_HERSHEY_PLAIN
    while True:
        
            ret_val, frame = cam.read() # Read each frame from camera
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) # Gray scale image
            
            red_img = frame.copy()
            # set blue and green channels to 0
            red_img[:, :, 0] = 0
            red_img[:, :, 1] = 0
            #Look for faces
            faces = face_cascade.detectMultiScale(gray, 1.1, 4)
            ar_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
            param = aruco.DetectorParameters_create()
            corners, ids, rejectedImgPoints = aruco.detectMarkers(frame, ar_dict, parameters=param)
            ar_gray = aruco.drawDetectedMarkers(frame, corners)
            if(len(corners) > 0):
                Xcord = [corners[0][0][0][0], corners[0][0][2][0]]
                Ycord = [corners[0][0][0][1], corners[0][0][2][1]]
                pixel_size = corners[0][0][0][0] - corners[0][0][1][0]
                distance = FOCAL_LENGTH * KNOWN_WIDTH_INCHES / pixel_size
                print("Distance to Object "+ str(distance) +"\n")
                x = midPointCalculation(Xcord)
                y = midPointCalculation(Ycord)
                
                center = (int(x), int(y))
                #print(str(x) +","+ str(y))
                LEFT_BOUND = 215
                RIGHT_BOUND = 430
                if x < LEFT_BOUND:
                    pass
                elif x > RIGHT_BOUND:
                    pass
                else:
                    pass
                print("\n")
                
                cv2.circle(gray, center, 5, BLUE, thickness=-1)
                
            #Look for faces
            if len(faces) != 0:
                cv2.putText(red_img,"FACE DETECTED", (250, 435), font, 2, (255, 255, 255))
            for (x, y, w, h) in faces:
                cv2.rectangle(red_img, (x, y), (x+w, y+h), (255, 0, 0), 2)
                
            cv2.imshow("Vision", gray)
            if cv2.waitKey(1) == 27:
                break  # esc to quit
    cam.release() # Release capture
    cv2.destroyAllWindows() # Close windows
    
    
def midPointCalculation(points):
    return (points[0] + points[1])/2

def main():
    show_webcam(mirror=True)


if __name__ == '__main__':
    main()