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
import time
import math

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

#port_offset = [[-0.0288, 0.0151]]
port_offset = [[-0.029, 0.016]]
 
class state(Enum):
    INITIALIZE = 0
    MOVE_POS = 1
    DEFAULT_CHARGE_POS = 2
    SEARCH_ALIGN = 3
    CHARGE_POS = 4

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
        self.theta = None

    
    def set_detect(self, on_off):
        self.activeDetect = on_off

    def set_detect_ref_img(self, id):
        self.detect.setRefImg(imgId = id)

    def set_offset(self, num):
        self.offset = int(num)

    def get_qrCenter(self):
        return self.qrCodeX, self.qrCodeY   
    
    def get_theta(self):
        return self.theta
    
    def run(self):
        if self.isVisual == True:
            cv2.namedWindow('Vision', cv2.WINDOW_NORMAL)

        while True:           
            frame = self.cam.getColorImage()
            
            if self.activeDetect == True:
                thread_lock.acquire()
                self.qrCodeX, self.qrCodeY, self.theta = self.detect.findQRcode(frame)
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
        self.arm.clean_error()
        self.arm.set_self_collision_detection(1)   

        # state machine states
        self.state_machine = [False, False, False, False, False]        

        #parameter get from ros
        self.action = True   #system get new charging mission, set this parameter to true, arm change position from sleep to move pose
        self.mobile_on_spot = True
        self.qrCodeId = 0

        #parameter for target QR code
        self.offset = offset

    def get_QRcenter(self, lineSpeed):
        qrX, qrY = self.camThread.get_qrCenter()
        while qrX == None or qrY == None:
            self.arm.set_tool_position(z= -2, speed=lineSpeed, is_radian=False, wait=True)
            qrX, qrY = self.camThread.get_qrCenter()
        
        return qrX, qrY

    def run(self):
        # start camera threading
        self.camThread.start()

        # default settings
        armState = state.INITIALIZE  
        angleSpeed = 10  # degree/s  
        lineSpeed_norm = 40 # mm/s  
        lineSpeed_slow = 8 # mm/s

        while True:
            '''
            state machine first part
            state changes
            '''
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
                    armState = state.SEARCH_ALIGN    

            elif armState == state.SEARCH_ALIGN:
                if self.state_machine[3] == True:
                    armState = state.CHARGE_POS           

            '''
            state machine first part
            state action
            '''
            if armState == state.INITIALIZE and self.state_machine[0] == False:
                self.arm.reset(speed=angleSpeed, wait=True)
                self.state_machine[0] = True

            elif armState == state.MOVE_POS and self.state_machine[1] == False:
                self.arm.set_servo_angle(angle=[30, -30, 0, 0, 30, 0], speed=angleSpeed, wait=True)    
                self.state_machine[1] = True   

            elif armState == state.DEFAULT_CHARGE_POS and self.state_machine[2] == False:           
                self.arm.set_servo_angle(angle=[0, 0, 0, 0, -90, 0], relative = True, speed=angleSpeed, wait=True)
                #print(self.arm.get_position())
                self.arm.set_tool_position(z = 25, speed=lineSpeed_norm, is_radian=False, wait=True)
                #print(self.arm.get_position())                
                self.state_machine[2] = True

            elif armState == state.SEARCH_ALIGN and self.state_machine[3] == False:
                self.arm.set_servo_angle(angle=[0, 0, 0, 0, 0, -6], relative = True, speed=angleSpeed, wait=True)
                '''
                use while loop to make sure get something
                '''
                qrX, qrY = self.get_QRcenter(lineSpeed_slow)
                centPos = self.camThread.cam.getCoordinate(qrX, qrY)
                print("[+] Centering:{:.2}".format(1000 * centPos[0]))
                self.arm.set_tool_position(y = 1000 * centPos[0], speed = lineSpeed_norm, is_radian=False, wait=True)
                
                qrX, qrY = self.get_QRcenter(lineSpeed_slow)
                refRpos = self.camThread.cam.getCoordinate(qrX + self.offset, qrY)
                refLpos = self.camThread.cam.getCoordinate(qrX - self.offset, qrY)
                error = 1000 * refRpos[2] - 1000 * refLpos[2]

                print("[+] Roll angle")
                while error > 0.6 or error < -0.6:
                    pGain = error / 3
                    print("[+] error:{}, pGain:{:.5}".format(error, pGain))
                    self.arm.set_tool_position(roll = pGain, speed = abs(pGain), is_radian=False, wait=True)

                    time.sleep(0.1)
                    qrX, qrY = self.get_QRcenter(lineSpeed_slow)
                    refRpos = self.camThread.cam.getCoordinate(qrX + self.offset, qrY)
                    print("    refRpos:{:.5}, {:.5}, {:.5}".format(refRpos[0], refRpos[1], refRpos[2]))
                    refLpos = self.camThread.cam.getCoordinate(qrX - self.offset, qrY)
                    print("    refLpos:{:.5}, {:.5}, {:.5}".format(refLpos[0], refLpos[1], refLpos[2]))
                    error = 1000 * refRpos[2] - 1000 * refLpos[2]
                
                print("[+] Yaw angle")
                
                theta = self.camThread.get_theta()
                while theta > 0.6 or theta < -0.6:
                    pGain = theta / 5
                    print("[+] theta:{:.5}, pGain:{:.5}".format(theta, pGain))
                    self.arm.set_tool_position(yaw = -pGain, speed = abs(theta / 2), is_radian=False, wait=True)  
                    time.sleep(0.1) 
                    theta = self.camThread.get_theta()  

                print("[+] Robotarm Aligned")          
                
                self.state_machine[3] = True              

               
            elif armState == state.CHARGE_POS and self.state_machine[4] == False:
                target = port_offset[self.qrCodeId]
                qrX, qrY = self.get_QRcenter(lineSpeed_slow)
                centPos = self.camThread.cam.getCoordinate(qrX, qrY)
                print("[+] prev_pos {}, {}".format(centPos[0],centPos[1]))
                y_move = centPos[0] - target[0]
                x_move = centPos[1] - target[1]
                print("    move:{}, {}".format(y_move, x_move))
                
                self.arm.set_tool_position(x = -x_move * 1000, y = y_move * 1000, speed = lineSpeed_slow, is_radian=False, wait=True)    
                qrX, qrY = self.get_QRcenter(lineSpeed_slow)            
                centPos = self.camThread.cam.getCoordinate(qrX, qrY)
                print("    curr_pos {}, {}".format(centPos[0],centPos[1]))
                self.state_machine[4] = True  
            #print("[+] state: {}".format(armState))                    
            
        
        # camera threading
        self.camThread.join()

if __name__ == "__main__":
    xarm6 = armControl(isVisual = True, offset = 60)
    xarm6.run()