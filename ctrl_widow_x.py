
import socket
import serial as sr
import time
import pygame

import matplotlib.pyplot as plt
import copy
import numpy as np
import sys
import matplotlib.patches as patches

from collections import deque
sys.path.append('utils/')
from threading import Lock
from threadhandler import ThreadHandler


class widow_x():

    def __init__(self):
        self.MODE = "cylindrical"
        self.SET_CYLINDRICAL_MODE_CMD = [0xff, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x30, 0xcf], #Set 3D Cylindrical mode / straight wrist and go to home
        self.GO_HOME_CMD = [0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0xdf] #tem que verificar se está certo
        self.START_UP_CMD = [0xFF, 0x00, 0x00, 0x00, 0xC8, 0x00, 0xC8, 0x00, 0x00, 0x02, 0x00, 0x01, 0x00, 0x80, 0x00, 0x70, 0x7C]
        self.GO_SLEEP_CMD = [0xff, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x60, 0x9f]
        self.EMERGENCY_STOP_CMD = [0xff, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x11, 0xee]
        #LIMITES MÁXIMOS
        self.LIMITE_SUPERIOR_X = 0
        self.LIMITE_INFERIOR_X = 4059
        self.LIMITE_SUPERIOR_Y = 400
        self.LIMITE_INFERIOR_Y = 50
        self.LIMITE_SUPERIOR_Z = 350
        self.LIMITE_INFERIOR_Z = 20
        self.LIMITE_INFERIOR_WRIST_ANGLE = 60
        self.LIMITE_SUPERIOR_WRIST_ANGLE = 120
        self.LIMITE_INFERIOR_WRIST_ROTATE = 0
        self.LIMITE_SUPERIOR_WRIST_ANGLE = 1023
        self.LIMITE_INFERIOR_GRIPPER = 0
        self.LIMITE_SUPERIOR_GRIPPER = 512
        #LIMITES SEGUROS
        ###NECESSARIO ANALISE
        #VALORES DO PACKAGE DE COMUNICAÇÃO
        self.HEADER = 0xFF
        self.EXTENDED_BYTE = 0x00 #move arm to position
        self.BUTTON_BYTE = 0x00 #do nothing
        self.TIME = 2000 #time in miliseconds
        self.DELTA = 125 #delta value from the package. Range: 0 - 255

        #VARIAVEIS DE STATUS
        self.isConnected = False

    def connect(self):
        flagConnected = False
        comunicacaoSerial = sr.Serial('/dev/ttyUSB0',
                                        38400,
                                        stopbits = sr.STOPBITS_ONE,
                                        bytesize = sr.EIGHTBITS,
                                        parity = sr.PARITY_NONE,)
        self.comunicacoSerial = comunicacaoSerial
        flagConnected = startUp()
        if flagConnected:
            sendCmdWaitForReply(self.SET_CYLINDRICAL_MODE_CMD)
            time.sleep(1)
        self.isConnected = flagConnected
        return flagConnected


    def startUp(self):
        flagConnected = False
        interacao = 0
        while not flagConnected:
            self.comunicacaoSerial.write(self.START_UP_CMD)
            ret = self.comunicacaoSerial.readline()
            if ret == b'\xff\x03\x00\x00\xfcInterbotix Robot Arm Online.\r\n':
                flagConnected = True
                print("WidowX live")
                return flagConnected
            elif interacao>10:
                print("comunicação falhou")
                return flagConnected
            interacao = interacao + 1



    def sendCmdWaitForReply(self,cmd):
        CHECK_RESPONSE = "\xff\x03\x00\x00\xfc"
        flagResposta = False
        t0 = time.perf_counter()
        timeout = 1
        interacao = 0
        ret = b'0x00'
        while not flagResposta:
            while self.comunicacaoSerial.out_waiting > 0:
                print("aguardando enviar todos os bytes...")
            self.comunicacaoSerial.write(cmd)
            while self.comunicacaoSerial.out_waiting > 0:
                print("aguardando enviar todos os bytes...")
            print("enviando -> ", cmd)
            res = []
            time.sleep(0.5)
            while isRXBufferEmpty(self) and not(time.perf_counter() - t0 > timeout):
                time.sleep(0.001)
            while not isRXBufferEmpty(self) and ret != b'\xfc':
                ret = self.comunicacaoSerial.read()
                res.append(ret)
                time.sleep(0.01)
            print(res)
            flagResposta = True

    def isRXBufferEmpty(self):
        qtdeInput = self.comunicacaoSerial.in_waiting
        if qtdeInput > 0:
            return False
        else:
            return True

    def stopEmergency(self):
        self.comunicacaoSerial.write(self.EMERGENCY_STOP_CMD)
        print(self.comunicacaoSerial.readline())

    def goSleep(self):
        self.comunicacaoSerial.write(self.GO_SLEEP_CMD)
        print(self.comunicacaoSerial.readline())

    def goHome(self):
        self.comunicacaoSerial.write(self.GO_HOME_CMD)
        print(self.comunicacaoSerial.readline())

    #TODO: colocar os limites operacionais
    def verificaLimites(self,x,y,z,gripper, wrist_angle, wriste_rot):
        if(x < self.LIMITE_INFERIOR_X):
            x = self.LIMITE_INFERIOR_X
        elif(x>self.LIMITE_SUPERIOR_X):
            x = self.LIMITE_SUPERIOR_X
        if(y < self.LIMITE_INFERIOR_Y):
            y = self.LIMITE_INFERIOR_Y
        elif(y>self.LIMITE_SUPERIOR_Y):
            y = self.LIMITE_SUPERIOR_Y
        if(z < self.LIMITE_INFERIOR_Z):
            z = self.LIMITE_INFERIOR_Z
        elif(z>self.LIMITE_SUPERIOR_Z):
            z = self.LIMITE_SUPERIOR_Z
        if(gripper < self.LIMITE_INFERIOR_GRIPPER):
            gripper = self.LIMITE_INFERIOR_GRIPPER
        elif(gripper>self.LIMITE_SUPERIOR_GRIPPER):
            gripper = self.LIMITE_SUPERIOR_GRIPPER
        if(wrist_angle < self.LIMITE_INFERIOR_WRIST_ANGLE):
            wrist_angle = self.LIMITE_INFERIOR_WRIST_ANGLE
        elif(wrist_angle>self.LIMITE_SUPERIOR_WRIST_ANGLE):
            wrist_angle = self.LIMITE_SUPERIOR_WRIST_ANGLE
        if(wriste_rot < self.LIMITE_INFERIOR_WRIST_ROTATE):
            wriste_rot = self.LIMITE_INFERIOR_WRIST_ROTATE
        elif(wriste_rot>self.LIMITE_SUPERIOR_WRIST_ROTATE):
            wriste_rot = self.LIMITE_SUPERIOR_WRIST_ROTATE
        return x,y,z,gripper,wrist_angle,wriste_rot


    #TODO: colocar a validação reais
    def sendValue(self,x=0,y=250,z=225,gripper = 256,wrist = 90,wrist_rot = 512):
        #https://learn.trossenrobotics.com/arbotix/arbotix-communication-controllers/31-arm-link-reference.html
        print("enviando comando com posição")
        x,y,z,gripper,wrist,wrist_rot = verificaLimites(x,y,z,gripper,wrist,wrist_rot)
        posicoes = [x,y,z,wrist,wrist_rot,gripper]
        package = preparePackage(posicoes)
        sendCmdWaitForReply(package)


    def preparePackage(self,posicoes):
        package = []
        package.append(self.HEADER)
        for pos in posicoes:
            highByte = (int(pos) >> 8) & 0xFF
            lowByte = int(pos) & 0xFF
            package.append(highByte)
            package.append(lowByte)

        package.append(self.DELTA)
        package.append(self.BUTTON_BYTE)
        package.append(self.EXTENDED_BYTE)
        package.append(checkSum(package))
        print(package)
        return package


    def checkSum(package):
        soma = sum(package[1:-1])
        inv_check_sum = int(soma) & 0xFF
        checksum = 255 - inv_check_sum
        return checksum
