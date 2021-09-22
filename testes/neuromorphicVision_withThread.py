
import socket
import serial as sr
import time
import pygame
import utilsDVS128
import matplotlib.pyplot as plt
import copy
import numpy as np
import sys
import math
import matplotlib.patches as patches
from segmentationUtils import segmentationUtils
from collections import deque
sys.path.append('../../../general/')
from threading import Lock
from threadhandler import ThreadHandler



frameTime = 1000
HOST = ''
PORT = 8000
clock = pygame.time.Clock()



def aquisicaoEdvs():
    global udp, pol, x, y, ts, filaFrame,mutex, ts_LSB, ts_MSB
    msg, cliente = udp.recvfrom(5000000)
    vet = []
    for a in msg:
        vet.append(a)
    size = int(len(vet)/5)
    pol.extend(vet[ : size])
    x.extend(vet[size : 2 * size])
    y.extend(vet[2 * size : 3 * size])
    ts_LSB.extend(vet[3 * size : 4 * size])
    ts_MSB.extend(vet[4 * size : ])
    ts = list(map(lambda LSB, MSB: LSB + (MSB << 8), ts_LSB, ts_MSB))
    if sum(ts) >= frameTime:
        mutex.acquire()
        filaFrame.append([pol,x,y])
        pol, x, y, ts_LSB, ts_MSB = [], [], [], [], []
        mutex.release()



def main():
    threadAquisicao = ThreadHandler(aquisicaoEdvs)

    global imageDimensions
    global filaFrame
    global mutex
    mutex = Lock()
    filaFrame = deque()
    font = {'family': 'serif',
        'color':  'white',
        'weight': 'normal',
        'size': 8,
    }
    stop = False
    pygame.init()
    displayEvents = utilsDVS128.DisplayDVS128(128, 128)
    global udp
    udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp.bind((HOST, PORT))
    global pol, x, y, ts_LSB, ts_MSB
    pol, x, y, ts_LSB, ts_MSB = [], [], [], [], []
    rects = []
    texts = []
    detection = []
    a = '%' #motor 1
    b = '*' #motor 2
    pos_atual_x = 110
    pos_atual_y = 115
    global comunicacaoSerial
    comunicacaoSerial = sr.Serial('/dev/ttyACM0',115200, timeout=1)


    for e in pygame.event.get():
        if e.type == pygame.QUIT:
            stop = True

    threadAquisicao.start()

    while True:
        if len(filaFrame) > 0:
            count = len(filaFrame)
            mutex.acquire()
            for i in range(count):
                frame = filaFrame.popleft()
                #if i == count -1:
                displayEvents.plotEventsF(frame[0],frame[1],frame[2])
                img = displayEvents.frame
                watershedImage, markers, detection = segmentationUtils.watershed(img,
                                                                                '--neuromorphic',
                                                                                minimumSizeBox=0.5,
                                                                                smallBBFilter=True,
                                                                                centroidDistanceFilter = True,
                                                                                mergeOverlapingDetectionsFilter = True,
                                                                                flagCloserToCenter=True)
                imageDimensions = img.shape
                utilsDVS128.plotBoundingBox(displayEvents.gameDisplay, detection, displayEvents.m)
                distance, x_distance, y_distance = getError(detection)

                if x_distance is not None and x_distance < 0:
                    pos_atual_x += (abs(x_distance)/100)
                    if pos_atual_x >= 180:
                        pos_atual_x = 180
                elif x_distance is not None and x_distance > 0:
                    pos_atual_x -= (abs(x_distance)/100)
                    if pos_atual_x <= 0:
                        pos_atual_x = 0
                if y_distance is not None and y_distance > 0:
                    pos_atual_y += (abs(y_distance)/100)
                    if pos_atual_y >= 140:
                        pos_atual_y = 130
                elif y_distance is not None and y_distance < 0:
                    pos_atual_y -= (abs(y_distance)/100)
                    if pos_atual_y <= 0:
                        pos_atual_y = 0
                sendValue(pos_atual_x,pos_atual_y)
                pygame.display.update()
            mutex.release()
    udp.close()



def getError(detection):
    distanceToCenter = None
    x_distance = None
    y_distance = None
    if len(detection) > 0:
        distanceToCenter = math.sqrt(((detection[0][4]-imageDimensions[1]/2)**2)+((detection[0][5]-imageDimensions[0]/2)**2))
        x_distance = detection[0][5]-(imageDimensions[1]/2)
        y_distance = detection[0][4]-(imageDimensions[0]/2)
    return distanceToCenter,y_distance,x_distance
def sendValue(posicao_x,posicao_y,motor =None):
        comunicacaoSerial.write(b'$')
        comunicacaoSerial.write(b"'")
        comunicacaoSerial.write(b'%')
        comunicacaoSerial.write(bytes([int(posicao_x)]))
        comunicacaoSerial.write(b'*')
        comunicacaoSerial.write(bytes([int(posicao_y)]))
        comunicacaoSerial.write(b'[')

if __name__ == "__main__":
	main()
