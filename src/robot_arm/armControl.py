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
    STANDBY = 0
    MOVE_POS = 1
    DEFAULT_CHARGE_POS = 2


class armControl:
    def __init__(self):
        #intel realsense camera
        #self.cam = realSenseCamera()
        
        #connect xarm6 
        self.arm = XArmAPI('192.168.1.221')  # AC power supply IP
        self.arm.motion_enable(enable=True)
        self.arm.set_mode(0)
        self.arm.set_state(state=0)
        self.arm.set_self_collision_detection(1)
        self.arm.reset(speed=15, wait=True)   

        # state machine states
        self.state_machine = [False, False, False]        

        #parameter get from ros
        self.action = True   #system get new charging mission, set this parameter to true, arm change position from sleep to move pose
        self.mobile_on_spot = True
           
        

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
        armState = state.STANDBY  
        angleSpeed = 20  # degree/s  
        lineSpeed = 50 # mm/s  
        
        while True:
            # state changes
            if armState == state.STANDBY:
                if self.state_machine[0] == True and self.action == True:
                    armState = state.MOVE_POS
                    self.state_machine[0] = False
            elif armState == state.MOVE_POS:
                if self.state_machine[1] == True and self.mobile_on_spot == True:
                    armState = state.DEFAULT_CHARGE_POS
                    self.state_machine[1] == False


            # state action
            if armState == state.STANDBY and self.state_machine[0] == False:
                self.arm.reset(speed=angleSpeed, wait=True)
                self.state_machine[0] == True
            elif armState == state.MOVE_POS and self.state_machine[1] == False:
                self.arm.set_servo_angle(angle=[30, -45, 0, 0, 45, 0], speed=angleSpeed, wait=True)    
                self.state_machine[1] = True     
            elif armState == state.DEFAULT_CHARGE_POS and self.state_machine[2] == False:           
                self.arm.set_servo_angle(angle=[0, 0, 0, 0, -90, 0], relative = True, speed=angleSpeed, wait=True)
                print(self.arm.get_position())
                self.arm.set_tool_position(z=50, speed=lineSpeed, is_radian=False, wait=True)
                print(self.arm.get_position())
                
                self.state_machine[2] = True
                    
            #self.arm.set_tool_position(roll = 10, speed=angleSpeed, is_radian=False, wait=True) # for future use 

            #print("[+] arm state:{}".format(armState))


if __name__ == "__main__":
    xarm6 = armControl()
    xarm6.run()