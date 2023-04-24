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
    INSERT = 5

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
        thread_lock.acquire()
        qr_x = self.qrCodeX
        qr_y = self.qrCodeY 
        thread_lock.release()

        return qr_x, qr_y  
    
    def get_theta(self):
        theta = None
        while theta == None:
            thread_lock.acquire()
            theta = self.theta
            thread_lock.release()

        return theta
    
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
        self.depth_stat_pos = 0 #position before insert charging gun

        # state machine states
        self.state_machine = [False, False, False, False, False, False]        

        #parameter get from ros
        self.action = True   #system get new charging mission, set this parameter to true, arm change position from sleep to move pose
        self.mobile_on_spot = True
        self.qrCodeId = 0

        #parameter for target QR code
        self.offset = offset

    def get_QRcenter(self, lineSpeed):
        qrX = None
        qrY = None
        while qrX == None or qrY == None:
            #self.arm.set_tool_position(z= -2, speed=lineSpeed, is_radian=False, wait=True)
            qrX, qrY = self.camThread.get_qrCenter()
        
        return qrX, qrY

    def run(self):
        # start camera threading
        self.camThread.start()

        # default settings
        armState = state.INITIALIZE  

        # tuning parameters
        angleSpeed = 10  # degree/s  
        lineSpeed_norm = 40 # mm/s  
        lineSpeed_slow = 12 # mm/s

        # following parameter for make tool paraller to port
        roll_thr  = 0.8 # tool parallel to charging port
        roll_step_factor = 5
        roll_speed_factor = 2

        # following parameter for make tool paraller to port
        yaw_thr  = 0.6 # tool parallel to charging port
        yaw_step_factor = 2
        yaw_speed = 1

        #state.CHARGE_POS parameters
        dist_thr = 0.8
        move_factor = 2
        speed_factor = 1

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

            elif armState == state.CHARGE_POS:
                if self.state_machine[4] == True:
                    armState = state.INSERT           

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
                self.arm.set_tool_position(z = 25, speed=lineSpeed_norm, is_radian=False, wait=True)             
                self.state_machine[2] = True

            elif armState == state.SEARCH_ALIGN and self.state_machine[3] == False:
                '''
                use while loop to make sure get something
                1000 is to transfor m to mm
                '''
                qrX, qrY = self.get_QRcenter(lineSpeed_slow)
                centPos = self.camThread.cam.getCoordinate(qrX, qrY)
                print("[+] Centering move:{:.2}".format(1000 * centPos[0]))
                self.arm.set_tool_position(y = 1000 * centPos[0], speed = lineSpeed_slow, is_radian=False, wait=True)
                time.sleep(0.3)

                print("[+] Roll angle")
                nums_left = []
                nums_right = []
                while True:
                    qrX, qrY = self.get_QRcenter(lineSpeed_slow)
                    while len(nums_right) < 30 or len(nums_left) < 30:
                        refRpos = self.camThread.cam.getCoordinate(qrX + self.offset, qrY)
                        if abs(refRpos[2] - 0.0) > 1E-6 :
                            nums_right.append(refRpos[2])
                        refLpos = self.camThread.cam.getCoordinate(qrX - self.offset, qrY)
                        if abs(refLpos[2] - 0.0) > 1E-6:
                            nums_left.append(refLpos[2])
       

                    error = np.median(nums_right) * 1000 - np.median(nums_left) * 1000
                    nums_right.clear()
                    nums_left.clear()                   

                    if abs(error) < roll_thr:
                        break

                    step = error / roll_step_factor  
                    speed = max(abs(step / roll_speed_factor), 1.0)
                    print("[+] error:{}, step:{:.5}, speed:{:.5}".format(error, step, speed))                 
                    self.arm.set_tool_position(roll = step, speed = yaw_speed, is_radian=False, wait=True)                    
                    time.sleep(0.1)
                    
                
                print("[+] Yaw angle")  
                nums = []           
                while True:
                    theta = self.camThread.get_theta()
                    
                    """
                    因sensor精度,取中位數,若要更準確,可用別的filter
                    """
                    while len(nums) < 30:
                        nums.append(theta)
                    
                    theta = np.median(nums)
                    nums.clear()

                    if abs(theta) < yaw_thr:
                        break
                    step = theta / yaw_step_factor
                    print("[+] theta:{}, step:{:.5}, speed:{}".format(theta, step, speed))
                    self.arm.set_tool_position(yaw = -step, speed = 1, is_radian=False, wait=True)                      
                    time.sleep(0.1) 

                self.state_machine[3] = True  
                print("[+] Robotarm Aligned")             

               
            elif armState == state.CHARGE_POS and self.state_machine[4] == False:

                target = port_offset[self.qrCodeId]

                while True:
                    qrX, qrY = self.get_QRcenter(lineSpeed_slow)
                    centPos = self.camThread.cam.getCoordinate(qrX, qrY)
                    if abs(centPos[2] - 0.0) < 1E-6:
                        continue

                    y_error = (centPos[0] - target[0]) * 1000
                    x_error = (centPos[1] - target[1]) * 1000
                    if abs(y_error) < dist_thr and abs(x_error) < dist_thr:
                        break
                    speed = int(max(abs(y_error), abs(x_error))) * speed_factor
                    self.arm.set_tool_position(x = -x_error / move_factor, y = y_error / move_factor, speed = min(speed, lineSpeed_slow), is_radian=False, wait=True) 
                    
                self.state_machine[4] = True  
                print("[+] Robotarm Ready to Insert")     

            elif armState == state.INSERT and self.state_machine[5] == False:
                qrX, qrY = self.get_QRcenter(lineSpeed_slow)
                centPos = self.camThread.cam.getCoordinate(qrX, qrY)
                
                self.arm.set_tool_position(z = centPos[2] * 1000 - 170, speed = lineSpeed_norm, wait = True)
                print("[+] stereo camera limit reached")
                self.state_machine[5] = True  
        
        # camera threading
        self.camThread.join()

if __name__ == "__main__":
    #parameters
    offset_parameter = 60

    xarm6 = armControl(isVisual = True, offset = offset_parameter)
    xarm6.run()