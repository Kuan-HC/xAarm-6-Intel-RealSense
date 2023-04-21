#!/usr/bin/env python3
# -*- coding: utf-8 -*-


import pyrealsense2 as rs
import numpy as np
import cv2

from tools.realsense import realSenseCamera
from tools.detect import qrCodeDetect

if __name__ == "__main__":
    cam = realSenseCamera()
    track = qrCodeDetect()
    track.setRefImg(imgId = 0)

    while True:
        try:
            rgbImage = cam.getColorImage()   
                    
            X, Y, _ = track.findQRcode(rgbImage)
            cv2.namedWindow('Rgb image, ESC to quit', cv2.WINDOW_NORMAL)            
            
            if X != None and Y != None:  # 或者 None
                xw, yw, zw = cam.getCoordinate(X,Y)
                print("QRcode center: {},{}, coordinates:{}".format(X,Y, cam.getCoordinate(X,Y)))
                cv2.putText(rgbImage, " {:.2f} meter".format(cam.getDistance(X, Y)), (X - 20, Y - 20),cv2.FONT_HERSHEY_DUPLEX, 1, (0, 0, 255), 2)          
                cv2.circle(rgbImage, (X, Y), 3, (0, 0, 255), -1)     
            
            cv2.imshow('Rgb image, ESC to quit', rgbImage)

            key = cv2.waitKey(1)    
    
            if key & 0xFF == ord('q') or key == 27:
                cv2.destroyAllWindows()
                break

        except Exception as e:
            pass