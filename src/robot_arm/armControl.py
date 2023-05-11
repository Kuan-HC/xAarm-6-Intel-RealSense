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
    ROLL_CENTER = 3
    YAW_CHARGE_POS = 4
    INSERT = 5
    CHARGING = 6
    PULL = 7

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
        angleSpeed = 20 # degree/s  
        lineSpeed_norm = 30 # mm/s  
        lineSpeed_slow = 12 # mm/s
        alignDistance = 200 # mm

        sampleLen = 20
        # following parameter for make tool paraller to port
        roll_thr  = 0.8 # tool parallel to charging port
        roll_step_factor = 4
        roll_speed = 3

        # following parameter for make tool paraller to port
        yaw_thr  = 0.6 # tool parallel to charging port
        yaw_step_factor = 1
        yaw_speed = 3
        

        #state.YAW_CHARGE_POS parameters
        dist_thr = 0.8

        # last insert phase each step distance
        lastPhaseStep = 0.2
        movement = 0.0 #total movement in insert phase

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
                    armState = state.ROLL_CENTER    

            elif armState == state.ROLL_CENTER:
                if self.state_machine[3] == True:
                    armState = state.YAW_CHARGE_POS 

            elif armState == state.YAW_CHARGE_POS:
                if self.state_machine[4] == True:
                    armState = state.INSERT   
                    self.arm.set_mode(1)
                    self.arm.set_state(state=0) 
                    self.arm.set_self_collision_detection(0)         

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
                self.arm.set_servo_angle(angle=[0, 0, 0, 0, -90, 0], relative = True, speed=angleSpeed, wait=False)
                self.arm.set_tool_position(z = 25, speed=lineSpeed_norm, is_radian=False, wait=True)             
                self.state_machine[2] = True

            elif armState == state.ROLL_CENTER and self.state_machine[3] == False:
                '''
                use while loop to make sure get something
                1000 is to transfor m to mm
                '''
                print("[+] QR center to width center")
                qrX, qrY = self.get_QRcenter(lineSpeed_slow)
                centPos = self.camThread.cam.getCoordinate(qrX, qrY)
                self.arm.set_tool_position(y = 1000 * centPos[0], speed = lineSpeed_slow, is_radian=False, wait=True)

                print("[+] Roll angle")     
                time.sleep(2.0)            
                while True:
                    qrX, qrY = self.get_QRcenter(lineSpeed_slow)
                    refRpos = 0.0
                    refLpos = 0.0
                    
                    for i in range(sampleLen):
                        rMeasure = 0.0
                        lMeasure = 0.0
                        while rMeasure < 0.1 or lMeasure < 0.1:
                            rMeasure = self.camThread.cam.getCoordinate(qrX + self.offset, qrY)[2]
                            lMeasure = self.camThread.cam.getCoordinate(qrX - self.offset, qrY)[2]                        
                        refRpos += rMeasure
                        refLpos += lMeasure
                    
                    refRpos /= sampleLen
                    refLpos /= sampleLen                                       
                    error = refRpos * 1000 - refLpos * 1000               
                    if abs(error) < roll_thr:
                        break

                    step = round((error / roll_step_factor), 1)
                    print("    error:{:.5}, step:{:.2}, speed:{}".format(error, step, roll_speed))                 
                    self.arm.set_tool_position(roll = step, speed = roll_speed, is_radian=False, wait=True)                    
                    time.sleep(0.1)   

                print("[+] QR center to image center")
                time.sleep(1.0)
                qrX, qrY = self.get_QRcenter(lineSpeed_slow)
                centPos = self.camThread.cam.getCoordinate(qrX, qrY)                
                self.arm.set_tool_position(x = -1000 * centPos[1], y = 1000 * centPos[0], speed = lineSpeed_slow, is_radian=False, wait=True)  

                print("[+] Move forward to {} mm".format(alignDistance))
                qrX, qrY = self.get_QRcenter(lineSpeed_slow)
                centPos = self.camThread.cam.getCoordinate(qrX, qrY)
                movement = centPos[2] * 1000 - alignDistance
                self.arm.set_tool_position(z = movement, speed = lineSpeed_norm, wait = True)

                self.state_machine[3] = True           

               
            elif armState == state.YAW_CHARGE_POS and self.state_machine[4] == False:
                print("[+] Yaw angle")     
                time.sleep(1.0)       
                while True:
                    theta = 0.0
                    
                    for i in range(sampleLen):
                        theta += self.camThread.get_theta()
                    theta /= sampleLen
                                    
                    if abs(theta) < yaw_thr:
                        break
                    step = round((theta / yaw_step_factor),1)
                    print("    theta:{:.4}, step:{:.2}, speed:{}".format(theta, step, yaw_speed))
                    self.arm.set_tool_position(yaw = -step, speed = yaw_speed, is_radian=False, wait=True)                      
                    time.sleep(0.1) 

                print("[+] Move to charging port")
                target = port_offset[self.qrCodeId]

                time.sleep(0.5)
                while True:
                    qrX, qrY = self.get_QRcenter(lineSpeed_slow)
                    centPos = self.camThread.cam.getCoordinate(qrX, qrY)
                    if abs(centPos[2] - 0.0) < 1E-6:
                        continue

                    y_error = (centPos[0] - target[0]) * 1000
                    x_error = (centPos[1] - target[1]) * 1000
                    if abs(y_error) < dist_thr and abs(x_error) < dist_thr:
                        break
                    
                    speed = min(int(max(abs(y_error), abs(x_error))), lineSpeed_slow)
                    print("    tool coordindate x:{:.5}, y:{:.5}, speed:{}".format(-x_error, y_error, max(speed, 1)))
                    self.arm.set_tool_position(x = -x_error, y = y_error, speed = max(speed, 1), is_radian=False, wait=True) 
                    
                self.state_machine[4] = True  
                print("[+] Robotarm Ready to Insert")     

            elif armState == state.INSERT and self.state_machine[5] == False:                              
                print("[+] insert ")
                '''
                Code below is just for test
                '''
                
                time.sleep(1)
                for i in range(400):
                    time.sleep(0.025)
                    self.arm.set_servo_cartesian_aa([0, 0, lastPhaseStep, 0, 0, 0], is_tool_coord=True, wait=False)
                    #print('set_servo_cartesian_aa, code={}, i={}, step={}'.format(code, i, lastPhaseStep))
                    movement += lastPhaseStep

                self.state_machine[5] = True  
        
        # camera threading
        self.camThread.join()

if __name__ == "__main__":
    #parameters
    offset_parameter = 60

    xarm6 = armControl(isVisual = True, offset = offset_parameter)
    xarm6.run()