# -*- coding: utf-8 -*-
"""
Last Updated on October 7 2019

@author: Thomas Talasco
"""

import sys
import cv2
from cv2 import aruco

#Import AR Dictionary
ar_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
#Verify correct num of args
if len(sys.argv) != 3:
    print("Invalid args - Usage: marker_id sidelengthinches")
else:
    #Verify correct format
    try:
        marker_id = int(sys.argv[1])
        if marker_id > 249 or marker_id < 0:
            sys.exit(1) #Throw exception on bad ID
    except:
        print("Invalid Marker ID, Valid ID 0-249")
    try:
        inches = int(sys.argv[2])
    except:
        print("Inches arg must be an integer")
    marker_size = inches * 96 # Get size in pixels
    if marker_size < 6:
        print("Cannot create 6x6 Marker with a size of 5x5 or smaller")
    else:
        #Write to file
        ar_marker = aruco.drawMarker(ar_dict, marker_id, marker_size)
        cv2.imwrite('ar'+ str(marker_id) +'.png', ar_marker)