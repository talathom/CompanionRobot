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
    while True:
        
            ret_val, frame = cam.read() # Read each frame from camera
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            ar_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
            param = aruco.DetectorParameters_create()
            corners, ids, rejectedImgPoints = aruco.detectMarkers(frame, ar_dict, parameters=param)
            gray = aruco.drawDetectedMarkers(frame, corners)
            if mirror: 
                frame = cv2.flip(frame, 1)
            if(len(corners) > 0):
                
                Xcord = [corners[0][0][0][0], corners[0][0][1][0], corners[0][0][2][0]]
                Ycord = [corners[0][0][0][1], corners[0][0][1][1], corners[0][0][2][1]]
                
                x = centerPointCalculation(Xcord)
                y = centerPointCalculation(Ycord)
                
                center = (int(x), int(y))
                print(str(x) +","+ str(y))
                LEFT_BOUND = 215
                RIGHT_BOUND = 430
                if x < LEFT_BOUND:
                    print("LEFT")
                elif x > RIGHT_BOUND:
                    print("RIGHT")
                else:
                    print("STRAIGHT")
                print("\n")
                #center = (corners[0][0][0][0] + (corners[0][0][1][0] - corners[0][0][0][0])/2, corners[0][0][1][1] + (corners[0][0][0][1] - corners[0][1][1])/2)
                cv2.circle(gray, center, 5, BLUE, thickness=-1)
            cv2.imshow("Vision", gray)
            if cv2.waitKey(1) == 27:
                break  # esc to quit
    cam.release() # Release capture
    cv2.destroyAllWindows() # Close windows
    
def centerPointCalculation(points):
    
    TOLERANCE = 40
    coordinate = -1
    difference = points[0] - points[1]
    if difference < TOLERANCE:
        difference = abs(points[0] - points[2])
        if points[2] < points[0]:
            coordinate = points[2] + difference/2
        else:
            coordinate = points[0] + difference/2
    else:
        if points[1] < points[0]:
            coordinate = points[1] + difference/2
        else:
            coordinate = points[0] + difference/2
    return coordinate
    


def main():
    show_webcam(mirror=True)


if __name__ == '__main__':
    main()