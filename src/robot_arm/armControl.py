#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys

#import realsense camera
from tools.realsense import realSenseCamera
from tools.detect import qrCodeDetect

# import xarm sdk
xarm_pkg = os.path.join(os.path.dirname(__file__), 'tools/xArm-Python-SDK')
sys.path.append(xarm_pkg)
from xarm.wrapper import XArmAPI

# import camera packages
import cv2

# import enumerator

from enum import Enum
 
class state(Enum):
    INITIALIZE = 0
    MOVE_POS = 1
    DEFAULT_CHARGE_POS = 2
    SEARCH_QR_CODE = 3


class armControl:
    def __init__(self):
        #intel realsense camera
        self.cam = realSenseCamera()
        
        #QR code detector
        self.detect = qrCodeDetect()
        
        #connect xarm6 
        self.arm = XArmAPI('192.168.1.221')  # AC power supply IP
        self.arm.motion_enable(enable=True)
        self.arm.set_mode(0)
        self.arm.set_state(state=0)
        self.arm.set_self_collision_detection(1)   

        # state machine states
        self.state_machine = [False, False, False]        

        #parameter get from ros
        self.action = True   #system get new charging mission, set this parameter to true, arm change position from sleep to move pose
        self.mobile_on_spot = True
        self.qrCodeId = 0
           

    def run(self):
        armState = state.INITIALIZE  
        angleSpeed = 15  # degree/s  
        lineSpeed = 40 # mm/s  

        isVisual = True # display real time image or not
        if isVisual == True:
                cv2.namedWindow('Vision', cv2.WINDOW_NORMAL)
        
        while True:
            # get image
            image = self.cam.getColorImage()  

            # state changes
            if armState == state.INITIALIZE:
                if self.state_machine[0] == True and self.action == True:
                    armState = state.MOVE_POS                    

            elif armState == state.MOVE_POS:
                if self.state_machine[1] == True and self.mobile_on_spot == True:
                    armState = state.DEFAULT_CHARGE_POS

            elif armState == state.DEFAULT_CHARGE_POS:
                if self.state_machine[2] == True:
                    self.detect.setRefImg(imgId = self.qrCodeId) 
                    armState = state.SEARCH_QR_CODE                    


            # state action
            if armState == state.INITIALIZE and self.state_machine[0] == False:
                self.arm.reset(speed=angleSpeed, wait=True)
                self.state_machine[0] = True

            elif armState == state.MOVE_POS and self.state_machine[1] == False:
                self.arm.set_servo_angle(angle=[30, -45, 0, 0, 45, 0], speed=angleSpeed, wait=True)    
                self.state_machine[1] = True   

            elif armState == state.DEFAULT_CHARGE_POS and self.state_machine[2] == False:           
                self.arm.set_servo_angle(angle=[0, 0, 0, 0, -90, 0], relative = True, speed=angleSpeed, wait=True)
                #print(self.arm.get_position())
                self.arm.set_tool_position(z=50, speed=lineSpeed, is_radian=False, wait=True)
                #print(self.arm.get_position())                
                self.state_machine[2] = True

            
            if isVisual == True:
                cv2.imshow('Vision', image)
                cv2.waitKey(1)

            print("[+] state: {}".format(armState))
                    
            #self.arm.set_tool_position(roll = 10, speed=angleSpeed, is_radian=False, wait=True) # for future use 



if __name__ == "__main__":
    xarm6 = armControl()
    xarm6.run()