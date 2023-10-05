#!/usr/bin/env python3
# -*- coding: utf-8 -*-

'''
 @ author  Kuan hsun chen
 @ version 2.0, 18th, Apr, 2023
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

from pynput import keyboard

from tools.xArm_Python_SDK.xarm.wrapper import XArmAPI

# Final insert position regards to qr code center
port_offset = [[-0.05666, -0.00468]]  # 若要向右移動-〉第0項 負值變大


'''
 define state machine states
'''
class state(Enum):
    STANDBY = 0
    DEFAULT_CHARGE_POS = 1
    ROLL = 2
    PITCH = 3
    YAW = 4
    ALIGN = 5 
    PLUG = 6
    GUM_LOCKING = 7
    LOCKED = 8
    GUM_UNLOCKING = 9
    UNPLUG = 10
    INITIALIZE = 255

thread_lock = threading.Lock()


class camera_thread(threading.Thread):
    def __init__(self, isVisual):
        super(camera_thread, self).__init__()

        self.cam = realSenseCamera()  # Intel realsense camera
        self.detect = qrCodeDetect()  # QR code detector

        # paramaters
        self.isVisual = isVisual
        self.activeDetect = False
        self.offset_H = 0
        self.offset_V = 0
        self.offset_V_Up = 0

        # return values
        self.qrCodeX = None
        self.qrCodeY = None
        self.theta = None

    
    def set_detect(self, on_off):
        self.activeDetect = on_off

    def set_detect_ref_img(self, id):
        self.detect.setRefImg(imgId=id)

    def set_H_offset(self, num):
        self.offset_H = int(num)

    def set_V_offset(self, num):
        self.offset_V = int(num)
        self.offset_V_Up = int(num / 2)

    def get_qrCenter(self):
        thread_lock.acquire()
        qr_x = self.qrCodeX
        qr_y = self.qrCodeY
        thread_lock.release()

        return qr_x, qr_y

    def get_theta(self):
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
                    # QR code center red point
                    cv2.circle(frame, (self.qrCodeX, self.qrCodeY), 3, (0, 0, 255), -1)
                    # left and right refference points for roll angle，color: blue
                    cv2.circle(frame, (self.qrCodeX + self.offset_H, self.qrCodeY), 3, (255, 0, 0), -1)
                    cv2.circle(frame, (self.qrCodeX - self.offset_H, self.qrCodeY), 3, (255, 0, 0), -1)
                    # up and down refference points for pitch angle，color: green
                    cv2.circle(frame, (self.qrCodeX, self.qrCodeY - self.offset_V_Up), 3, (0, 255, 0), -1)
                    cv2.circle(frame, (self.qrCodeX, self.qrCodeY + self.offset_V), 3, (0, 255, 0), -1)

            if self.isVisual == True:
                cv2.imshow('Vision', frame)
                cv2.waitKey(1)
            

class armControl:
    def __init__(self,isVisual = True, offset_H = 50, offset_V = 50):
        #camera thread
        self.camThread = camera_thread(isVisual)
        
        #connect xarm6 
        self.arm = XArmAPI('192.168.1.221')  # AC power supply IP
        self.arm.motion_enable(enable=True)
        self.arm.set_mode(0)
        self.arm.set_state(state=0)
        self.arm.clean_error()
        self.arm.set_collision_sensitivity(2)

        # default settings
        self.armState = state.INITIALIZE 

        #parameter get from ros communication
        self.action = True   #system get new charging mission, set this parameter to true, arm change position from sleep to move pose
        self.qrCodeId = 0

        # parameter for target QR code
        self.offset_H = offset_H
        self.offset_V = offset_V
        self.offset_V_Up = int(offset_V / 2)

        # keyboard
        self.listener = keyboard.Listener(on_press=self.on_press)
        self.listener.start()

        time.sleep(0.5)        

    def on_press(self, key):
        try:
            if key.char == 'c':
                self.action = False
            elif key.char == 'a':
                self.action = True

            print("robot state: {}".format(self.pub_system_msg.charging_robot_state ))
            
        except AttributeError:
            print('special key {0} pressed'.format(key))

    def get_QRcenter(self):
        
        return self.camThread.get_qrCenter()

    def run(self):
        # start camera threading
        self.camThread.start()

        # arm move speed
        angleSpeed = 20  # degree/s
        lineSpeed_norm = 20  # mm/s
        lineSpeed_slow = 10  # mm/s

        # roll and pitch will take 40 measurement, because intel realsens sense need longer
        # yaw take 60
        defaultDist = 300  # mm
        sampleLen = 15
        sampleLenLong = 60

        # following parameter for state Roll and Pitch
        roll_thr = 0.6  # tool parallel to charging port
        pitch_thr = 0.6
        angle_speed = 3

        # roll variables
        roll_move = False
        roll_dist = False
        # refRpos = np.array([.0, .0, .0])
        # refLpos = np.array([.0, .0, .0])
        refRightList = []
        refLeftList = []
        # rollCnt = 0

        # pitch variables
        pitch_move = False
        # upPos = np.array([.0, .0, .0])
        # downPos = np.array([.0, .0, .0])
        refUpList = []
        refDownList = []
        # pitchCnt = 0

        # following parameter for state Yaw
        yaw_thr = 0.6  # tool parallel to charging port
        yaw_step_factor = 1
        yaw_speed = 3
        yawDist = 280  # mm
        yaw_dist_move = False
        theta = 0.0
        thetaCnt = 0

        # align parameters
        align_dist_thr = 0.6

        # last insert phase each step distance
        plugStep = 0.1
        moveCnt = 0.0

        # counter how many times completed
        cnt = 0

        state_machine = [False, True, True, True, True, True, True, True, True, True, True]
        self.arm.set_tgpio_digital(ionum=0, value=0)
        # reset xarm6
        self.arm.reset(speed=angleSpeed, wait=True)
        # After reset, change state to standby
        self.armState = state.STANDBY


        while True:
            '''
            state machine first part
            when state can changes
            '''

            if self.armState == state.STANDBY:
                if state_machine[0] == True and self.action == True:
                    self.armState = state.DEFAULT_CHARGE_POS
                    state_machine = [True, False, False, False, False, False, False, False, False, False, False]

            elif self.armState == state.DEFAULT_CHARGE_POS:
                if self.action == False:
                    # self.camThread.set_detect(False)
                    self.armState = state.STANDBY
                    state_machine[0] = False
                elif state_machine[1] == True:
                    self.arm.set_tgpio_digital(ionum=0, value=1)
                    self.camThread.set_detect_ref_img(self.qrCodeId)
                    self.camThread.set_detect(True)
                    self.camThread.set_H_offset(self.offset_H)
                    self.camThread.set_V_offset(self.offset_V)
                    self.armState = state.ROLL
                    print("[+] Arm state: Roll")
                    roll_move = False
                    roll_dist = False
                    refRightList.clear()
                    refLeftList.clear()
                    time.sleep(1.5)

            elif self.armState == state.ROLL:
                if self.action == False:
                    self.arm.set_tgpio_digital(ionum=0, value=0)
                    self.camThread.set_detect(False)
                    self.armState = state.STANDBY
                    state_machine[0] = False
                elif state_machine[2] == True:
                    self.armState = state.PITCH
                    print("[+] Arm state: Pitch")
                    pitch_move = False
                    refUpList.clear()
                    refDownList.clear()
                    time.sleep(0.5)

            elif self.armState == state.PITCH:
                if self.action == False:
                    self.arm.set_tgpio_digital(ionum=0, value=0)
                    self.camThread.set_detect(False)
                    self.armState = state.STANDBY
                    state_machine[0] = False
                elif state_machine[3] == True:
                    self.armState = state.YAW
                    print("[+] Arm state: Yaw")
                    yaw_dist_move = False
                    theta = 0.0
                    thetaCnt = 0

            elif self.armState == state.YAW:
                if self.action == False:
                    self.arm.set_tgpio_digital(ionum=0, value=0)
                    self.camThread.set_detect(False)
                    self.armState = state.STANDBY
                    state_machine[0] = False
                elif state_machine[4] == True:
                    self.armState = state.ALIGN
                    print("[+] Arm state: Align")
                    time.sleep(1)

            elif self.armState == state.ALIGN:
                if self.action == False:
                    self.arm.set_tgpio_digital(ionum=0, value=0)
                    self.camThread.set_detect(False)
                    self.armState = state.STANDBY
                    state_machine[0] = False
                elif state_machine[5] == True:
                    self.arm.set_tgpio_digital(ionum=0, value=0)
                    self.armState = state.PLUG
                    self.arm.set_mode(1)
                    self.arm.set_state(state=0)
                    self.camThread.set_detect(False)
                    print("[+] Arm state: Plug")
                    moveCnt = 0.0
                    time.sleep(1.5)   # this must set to 1.5 for xarm to configure its mode

            elif self.armState == state.PLUG:
                if self.action == False:
                    self.arm.set_mode(0)
                    self.arm.set_state(state=0)
                    time.sleep(0.5)
                    self.armState = state.UNPLUG
                    print("[+] Arm state: UNPLUG")
                elif state_machine[6] == True:
                    self.arm.set_mode(0)
                    self.arm.set_state(state=0)
                    time.sleep(0.5)
                    self.armState = state.LOCKED
                    print("[+] Charging gum locked ")
                    lockStateInitalized = False
                    charging = False
                    
            elif self.armState == state.LOCKED:
                if state_machine[8] == True:
                    self.armState = state.UNPLUG
                    print("[+] Arm state: UNPLUG")

            elif self.armState == state.UNPLUG:
                if state_machine[10] == True:
                    self.armState = state.STANDBY
                    # set this to standyby state to False, so the arm could execute action in STANDBY
                    state_machine[0] = False

                    # show how many cycles has performed for robust test
                    cnt += 1
                    print("[+] Total {} cycles completed\n".format(cnt))

            '''
            state machine second part
            state action
            '''
            if self.armState == state.STANDBY and state_machine[0] == False:
                self.arm.set_servo_angle(angle=[30, -30, 0, 0, 30, 0], speed=angleSpeed, wait=True)
                print("[+] Arm state: Standby")
                state_machine[0] = True

            elif self.armState == state.DEFAULT_CHARGE_POS and state_machine[1] == False:
                print("[+] Arm state: Default charge pos")
                self.arm.set_servo_angle(angle=[0, 0, 0, 0, -90, 0], relative=True, speed=angleSpeed, wait=True)
                state_machine[1] = True

            elif self.armState == state.ROLL and state_machine[2] == False:
                qrX, qrY = self.get_QRcenter()
                if qrX == None or qrY == None:
                    continue

                
                if roll_move == False:
                    centPos = self.camThread.cam.getCoordinate(qrX, qrY)
                    # 取得資料有問題，重來過
                    if centPos[2] < 0.1:
                        continue
                    
                    if abs(1000 * centPos[1]) > 20 or abs(1000 * centPos[0]) > 20:
                        print("    Move QR center to image center")

                        speed = math.ceil(max(abs(1000 * centPos[1]), abs(1000 * centPos[0])) / 2)
                        speed = min(speed, lineSpeed_norm)                   
                        print("    move speed: {} mm/s, x moement: {:.2} mm".format(speed, -1000 * centPos[1]))
                        self.arm.set_tool_position(x=-1000 * centPos[1], y=1000 * centPos[0],
                                                    speed=speed, is_radian=False, wait=True)
                        time.sleep(1.5)
                    
                    if roll_dist == False:
                        centPos = self.camThread.cam.getCoordinate(qrX, qrY)
                        if centPos[2] < 0.1:
                            continue

                        distQr = round((centPos[2] * 1000), 1)
                        if abs(distQr) > 500.0:
                            continue

                        print("    Distance to QR code center: {} mm ".format(distQr))
                        dist =  distQr - defaultDist
                        speed = math.ceil(abs(dist)/1.5)
                        speed = min(speed, lineSpeed_norm)
                        if dist > 0.0:
                            print("    Move forward {} mm to default detect distance {} mm".format(dist, defaultDist))
                            self.arm.set_tool_position(z=dist, speed=speed, wait=True)
                            time.sleep(1)

                        roll_dist = True
                        time.sleep(1.5)

                    roll_move = True
                    print("    Get roll angle")
                
                else:
                    rMeasure = self.camThread.cam.getCoordinate(qrX - self.offset_H, qrY)
                    lMeasure = self.camThread.cam.getCoordinate(qrX + self.offset_H, qrY)
                    if rMeasure[2] < 0.1 or lMeasure[2] < 0.1:
                        continue
                    
                    if len(refRightList) < sampleLen:
                        # print("left:{}".format(lMeasure))
                        refRightList.append(rMeasure)
                        refLeftList.append(lMeasure)
                        continue  

                    refRposMost = max(refRightList, key = refRightList.count)
                    refLposMost = max(refLeftList, key = refLeftList.count)
                    # print("max Left :{}, \nmax Right:{}".format(refLposMost, refRposMost))

                    error = math.atan((refLposMost[2] * 1000 - refRposMost[2] * 1000) / (refLposMost[0] * 1000 - refRposMost[0] * 1000)) / math.pi * 180
                    if abs(error) > 20.0:
                        print("      error degree:{:.5}, seems not right, retake".format(error))
                        roll_move = False
                        refRightList.clear()
                        refLeftList.clear()
                        continue

                    if abs(error) < roll_thr:
                        print("      error degree:{:.5}".format(error))
                        print("      roll adjustment completed")
                        state_machine[2] = True
                        continue

                    if abs(error) > 1.4:
                        step = round(error, 1)
                    else:
                        step = round(error/2, 1)

                    speed = math.ceil(abs(error)/2)
                    speed = min(speed, angle_speed)

                    print("      error degree:{:.5}, step:{:.2}, speed:{}".format(error, step, speed))
                    self.arm.set_tool_position(roll=step, speed=speed, is_radian=False, wait=True)
                    roll_move = False
                    refRightList.clear()
                    refLeftList.clear()
                    time.sleep(1.5)


            elif self.armState == state.PITCH and state_machine[3] == False:
                qrX, qrY = self.get_QRcenter()
                if qrX == None or qrY == None:
                    continue

                if pitch_move == False:
                    centPos = self.camThread.cam.getCoordinate(qrX, qrY)
                    if centPos[2] < 0.1:
                        continue

                    if abs(1000 * centPos[1]) > 20 or abs(1000 * centPos[0]) > 20:
                        print("    Move QR center to image center")

                        speed = math.ceil(max(abs(1000 * centPos[1]), abs(1000 * centPos[0])) / 2)
                        speed = min(speed, lineSpeed_norm)                   
                        print("    move speed: {} mm/s, x moement: {:.2} mm".format(speed, -1000 * centPos[1]))
                        self.arm.set_tool_position(x=-1000 * centPos[1], y=1000 * centPos[0],
                                                    speed=speed, is_radian=False, wait=True)
                        time.sleep(1.5)
                    
                    pitch_move = True
                    print("    Get pitch angle")
                
                else:
                    upMeasure = self.camThread.cam.getCoordinate(qrX, qrY - self.offset_V_Up)
                    downMeasure = self.camThread.cam.getCoordinate(qrX, qrY + self.offset_V)
                    if upMeasure[2] < 0.1 or downMeasure[2] < 0.1:
                        continue
                    
                    if len(refUpList) < sampleLen:
                        # print("up:{}".format(upMeasure))
                        refUpList.append(upMeasure)
                        refDownList.append(downMeasure)
                        continue

                    upPosMost = max(refUpList, key = refUpList.count)
                    downPosMost = max(refDownList, key = refDownList.count)
                    # print("max Up  :{}, \nmax Down:{}".format(upPosMost, downPosMost))
                    
                    error = math.atan((upPosMost[2] * 1000 - downPosMost[2] * 1000) / (upPosMost[1] * 1000 - downPosMost[1] * 1000)) / math.pi * 180
                    if abs(error) > 5.0:
                        print("      error degree:{:.5}, seems not right, retake".format(error))
                        pitch_move = False
                        refUpList.clear()
                        refDownList.clear()
                        continue
                    
                    if abs(error) < pitch_thr:
                        print("      error degree:{:.5}".format(error))
                        print("      pitch adjustment completed")
                        state_machine[3] = True
                        continue

                    if abs(error) > 1.4:
                        step = round(error, 1)
                    else:
                        step = round(error/2, 1)

                    speed = math.ceil(abs(error)/2)
                    speed = min(speed, angle_speed)

                    print("      error degree:{:.5}, step:{:.2}, speed:{}".format(error, step, speed))
                    self.arm.set_tool_position(pitch=step, speed=speed, is_radian=False, wait=True)
                    refUpList.clear()
                    refDownList.clear()
                    pitch_move = False
                    time.sleep(1.5)


            elif self.armState == state.YAW and state_machine[4] == False:
                if yaw_dist_move == False:
                    qrX, qrY = self.get_QRcenter()
                    if qrX != None and qrY != None:
                        centPos = self.camThread.cam.getCoordinate(qrX, qrY)
                        if centPos[2] < 0.1:
                            continue
                        
                        print("    Move forward to {} mm".format(yawDist))
                        self.arm.set_tool_position(z=centPos[2] * 1000 - yawDist, speed=lineSpeed_slow, wait=True)
                        print("    Get Yaw angle")
                        time.sleep(1.0)
                        yaw_dist_move = True

                else:
                    if thetaCnt < sampleLenLong:
                        tmpTheta = self.camThread.get_theta()
                        if tmpTheta != None:
                            theta += tmpTheta
                            thetaCnt += 1
                    else:
                        theta /= sampleLenLong
                        if abs(theta) < yaw_thr:
                            state_machine[4] = True
                            print("      theta:{:.4}".format(theta))
                        else:
                            step = round((theta / yaw_step_factor), 1)
                            print("      theta:{:.4}, step:{:.2}, speed:{}".format(theta, step, yaw_speed))
                            self.arm.set_tool_position(yaw=-step, speed=yaw_speed, is_radian=False, wait=True)
                            thetaCnt = 0
                            theta = 0.0
                            time.sleep(0.5)


            elif self.armState == state.ALIGN and state_machine[5] == False:
                qrX, qrY = self.get_QRcenter()
                if qrX != None and qrY != None:
                    centPos = self.camThread.cam.getCoordinate(qrX, qrY)
                    if centPos[2] < 0.1:
                        continue

                    y_error = round((centPos[0] - port_offset[self.qrCodeId][0]) * 1000, 1)
                    x_error = round((centPos[1] - port_offset[self.qrCodeId][1]) * 1000, 1)

                    if abs(y_error) <= align_dist_thr and abs(x_error) <= align_dist_thr: 
                        print("    aligned x_error:{} y_error:{}".format(-x_error, y_error))
                        print("    Arm is ready to plug")
                        state_machine[5] = True
                    else:
                        maxDist = max(abs(y_error), abs(x_error))
                        if maxDist > 10.0:
                            speed = lineSpeed_slow
                        elif maxDist > 1.0:
                            speed = 1
                        else :
                            speed = 0.5
                        print("    tool end move x:{:.5}, y:{:.5}, speed:{}".format(-x_error, y_error, speed))
                        self.arm.set_tool_position(x=-x_error, y=y_error, speed=speed, is_radian=False, wait=True)
                        time.sleep(1.5)

            elif self.armState == state.PLUG and state_machine[6] == False: 
                '''
                An approximation switch is connect to tool digital input 1
                '''
                _, digitals = self.arm.get_tgpio_digital()
                toolDigInput_0 = digitals[0]

                if toolDigInput_0 == False:
                    self.arm.set_servo_cartesian_aa([0, 0, plugStep, 0, 0, 0], is_tool_coord=True, wait=False)
                    moveCnt += plugStep
                else:
                    print("    Plug completted ")
                    state_machine[6] = True

            elif self.armState == state.LOCKED and state_machine[8] == False:
                time.sleep(3)
                print("    Charing completed!")
                state_machine[8] = True

            elif self.armState == state.UNPLUG and state_machine[10] == False:
                dist = -moveCnt / 10
                print("    unplug distance:{}".format(dist))
                self.arm.set_tool_position(z = dist, speed=lineSpeed_norm, wait=True)
                state_machine[10] = True
  
        # camera threading
        self.camThread.join()

if __name__ == "__main__":
    #parameters

    xarm6 = armControl(isVisual = True, offset_H = 200, offset_V = 200)
    xarm6.run()