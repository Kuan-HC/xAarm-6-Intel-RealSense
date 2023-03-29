#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys

#import realsense camera
from tools.realsense import realSenseCamera

# import xarm sdk
xarm_pkg = os.path.join(os.path.dirname(__file__), 'tools/xArm-Python-SDK')
sys.path.append(xarm_pkg)
from xarm.wrapper import XArmAPI

# import camera packages
import cv2

# import enumerator

from enum import Enum
 
class state(Enum):
    INITIAL_POS = 0
    DEFAULT_POS = 1


class armControl:
    def __init__(self):
        #intel realsense camera
        #self.cam = realSenseCamera()

        #connect xarm6 
        self.arm = XArmAPI('192.168.1.218')
        self.arm.motion_enable(enable=True)
        self.arm.set_mode(0)
        self.arm.set_state(state=0)
        
        

    def cameraTest(self):
        x = 320
        y = 240
        while True:
    
            rgbImage = self.cam.getColorImage()

            cv2.namedWindow('Rgb image, ESC to quit', cv2.WINDOW_NORMAL)
        

            print(self.cam.getCoordinate(x,y))
            cv2.putText(rgbImage, " {:.2f} meter".format(self.cam.getDistance(x,y)), (x - 20, y - 20), cv2.FONT_HERSHEY_DUPLEX, 1 , (0,0,255), 2)
            cv2.circle(rgbImage, (320, 240), 3, (0,0,255), -1)

            cv2.imshow('Rgb image, ESC to quit', rgbImage)

            key = cv2.waitKey(1)    
    
            if key & 0xFF == ord('q') or key == 27:
                cv2.destroyAllWindows()
                break

    def run(self):
        armState = state.INITIAL_POS
        armInitialized = False
        

        # state changes
        while True:
            if armState == state.INITIAL_POS:
                if armInitialized == True:
                    armState = state.DEFAULT_POS



            # state action
            if armState == state.INITIAL_POS:
                if armInitialized == False:
                    self.arm.reset(wait=True)
                    armInitialized = True
                    print("[+] Xarm initialized")
            
            print("[+] arm state:{}".format(armState))


if __name__ == "__main__":
    xarm6 = armControl()
    xarm6.run()