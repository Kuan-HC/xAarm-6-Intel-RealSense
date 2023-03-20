#!/usr/bin/env python3
# -*- coding: utf-8 -*-


import pyrealsense2 as rs
import numpy as np
import cv2

from realsense import realSenseCamera

if __name__ == "__main__":
    cam = realSenseCamera()

    x = 320
    y = 240

    while True:
    
        rgbImage = cam.getColorImage()

        cv2.namedWindow('Rgb image, ESC to quit', cv2.WINDOW_NORMAL)
        

        print(cam.getCoordinate(x,y))
        cv2.putText(rgbImage, " {:.2f} meter".format(cam.getDistance(x,y)), (x - 20, y - 20), cv2.FONT_HERSHEY_DUPLEX, 1 , (0,0,255), 2)
        cv2.circle(rgbImage, (320, 240), 3, (0,0,255), -1)

        cv2.imshow('Rgb image, ESC to quit', rgbImage)


        key = cv2.waitKey(1)    
    
        if key & 0xFF == ord('q') or key == 27:
            cv2.destroyAllWindows()
            break

    