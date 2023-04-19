#!/usr/bin/env python3
# -*- coding: utf-8 -*-

'''
 @ author  Kuan hsun chen
 @ version 0.0, 18th, Apr, 2023
 @ email: k.h.chen33@gmail.com
 @ Git: www.github.com/Kuan-HC
'''

import os
import sys
import cv2
import threading
import numpy as np
from copy import deepcopy

# import enumerator
from enum import Enum

#import realsense camera and QRcode detector
from tools.realsense import realSenseCamera
from tools.detect import qrCodeDetect

# import xarm sdk
xarm_pkg = os.path.join(os.path.dirname(__file__), 'tools/xArm-Python-SDK')
sys.path.append(xarm_pkg)
from xarm.wrapper import XArmAPI

 
class state(Enum):
    INITIALIZE = 0
    MOVE_POS = 1
    DEFAULT_CHARGE_POS = 2
    SEARCH_QR_CODE = 3

thread_lock = threading.Lock()

class camera_thread(threading.Thread):
    def __init__(self, isVisual):
        super(camera_thread, self).__init__()
        
        self.cam = realSenseCamera()  #Intel realsense camera
        self.detect = qrCodeDetect()  #QR code detector
        # paramaters
        self.isVisual = isVisual
        self.activeDetect = False
        self.offset = 0

        # retur values
        self.qrCodeX = None
        self.qrCodeY = None

        # create a image, this will be used for QR code detection
        #self.frame = np.zeros((480, 640, 3), dtype=np.uint8)
    
    def set_detect(self, on_off):
        self.activeDetect = on_off

    def set_detect_ref_img(self, id):
        self.detect.setRefImg(imgId = id)

    def set_offset(self, num):
        self.offset = int(num)

    def get_qrCenter(self):
        return self.qrCodeX, self.qrCodeY   
    
    def run(self):
        if self.isVisual == True:
            cv2.namedWindow('Vision', cv2.WINDOW_NORMAL)

        while True:           
            frame = self.cam.getColorImage()
            
            if self.activeDetect == True:
                thread_lock.acquire()
                self.qrCodeX, self.qrCodeY, = self.detect.findQRcode(frame)
                thread_lock.release() 
                if self.qrCodeX != None and self.qrCodeY != None:  
                    cv2.circle(frame, (self.qrCodeX, self.qrCodeY), 3, (0, 0, 255), -1)
                    cv2.circle(frame, (self.qrCodeX + self.offset, self.qrCodeY), 3, (255, 0, 0), -1)
                    cv2.circle(frame, (self.qrCodeX - self.offset, self.qrCodeY), 3, (255, 0, 0), -1)                      
            
            if self.isVisual == True:
                cv2.imshow('Vision', frame)
                cv2.waitKey(1)
            

class armControl:
    def __init__(self, isVisual = True, offset = 50):
        #camera thread
        self.camThread = camera_thread(isVisual)
        
        #connect xarm6 
        self.arm = XArmAPI('192.168.1.221')  # AC power supply IP
        self.arm.motion_enable(enable=True)
        self.arm.set_mode(0)
        self.arm.set_state(state=0)
        self.arm.set_self_collision_detection(1)   

        # state machine states
        self.state_machine = [False, False, False, False]        

        #parameter get from ros
        self.action = True   #system get new charging mission, set this parameter to true, arm change position from sleep to move pose
        self.mobile_on_spot = True
        self.qrCodeId = 0

        #parameter for target QR code
        self.offset = offset
        

    def run(self):
        # start camera threading
        self.camThread.start()

        # default settings
        armState = state.INITIALIZE  
        angleSpeed = 15  # degree/s  
        lineSpeed = 40 # mm/s  

        while True:
            # state changes
            if armState == state.INITIALIZE:
                if self.state_machine[0] == True and self.action == True:
                    armState = state.MOVE_POS                    

            elif armState == state.MOVE_POS:
                if self.state_machine[1] == True and self.mobile_on_spot == True:
                    armState = state.DEFAULT_CHARGE_POS

            elif armState == state.DEFAULT_CHARGE_POS:
                if self.state_machine[2] == True:
                    self.camThread.set_detect_ref_img(self.qrCodeId)
                    self.camThread.set_detect(True)
                    self.camThread.set_offset(self.offset)
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

            elif armState == state.SEARCH_QR_CODE and self.state_machine[3] == False:
                '''
                use while loop to make sure get something
                '''
                qrX, qrY = self.camThread.get_qrCenter()
                while qrX == None or qrY == None:
                    qrX, qrY = self.camThread.get_qrCenter()

                centWcoord = self.camThread.cam.getCoordinate(qrX, qrY)
                
                print("[+] qrCodeCenter: {}, {}, {}".format(centWcoord[0], centWcoord[1], centWcoord[2]))                   
                                   
                            
                # if X != None and Y != None:
                #     xw, yw, zw = self.camThread.cam.getCoordinate(X, Y)  
                #     xw *= 1000  # transfer to mm
                #     yw *= 1000
                #     if abs(xw) > 1:
                #         print("[+] Targeting QR Code: tool coordinate y:{}".format(int(xw)))
                #         self.arm.set_tool_position(y = int(xw), speed=10, is_radian=False, wait=True)


                #     print("[+] x:{}, y:{}, armPosition:{}".format(xw,yw, self.arm.get_position()))
                #     cv2.circle(copyFrame, (X, Y), 3, (0, 0, 255), -1)

                # cv2.imshow('QR Code Detect', copyFrame)
                # cv2.waitKey(1)     

                self.state_machine[3] = True              
            
            #print("[+] state: {}".format(armState))
                    
            #self.arm.set_tool_position(roll = 10, speed=angleSpeed, is_radian=False, wait=True) # for future use 
        
        # camera threading
        self.camThread.join()

if __name__ == "__main__":
    xarm6 = armControl(isVisual = True, offset = 50)
    xarm6.run()