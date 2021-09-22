
import socket
from time import time
import pygame
import utilsDVS128
import matplotlib.pyplot as plt
import copy
import serial
import numpy as np
import sys
import math
import matplotlib.patches as patches
from segmentationUtils import segmentationUtils


frameTime = 5000
HOST = ''
PORT = 8000
clock = pygame.time.Clock()


def main():
    global imageDimensions
    font = {'family': 'serif',
        'color':  'white',
        'weight': 'normal',
        'size': 8,
    }
    global comunicacaoSerial
    comunicacaoSerial = serial.Serial('/dev/ttyACM0',115200, timeout=1)
    fig,axarr = plt.subplots(1)
    handle = None
    stop = False
    pygame.init()
    displayEvents = utilsDVS128.DisplayDVS128(128, 128)
    udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp.bind((HOST, PORT))
    pol, x, y, ts_LSB, ts_MSB = [], [], [], [], []
    count = []
    rects = []
    texts = []
    detection = []
    a = '%' #motor 1
    b = '*' #motor 2
    pos_atual_x = 110
    pos_atual_y = 115
    while not stop:
        t = time()

        for e in pygame.event.get():
            if e.type == pygame.QUIT:
                stop = True

        vet = []
        msg, cliente = udp.recvfrom(50000)
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
            displayEvents.plotEventsF(pol, x, y)
            img = displayEvents.frame
            imageDimensions = img.shape

            watershedImage, markers, detection = segmentationUtils.watershed(img,
                                                                            '--neuromorphic',
                                                                            minimumSizeBox=0.5,
                                                                            smallBBFilter=True,
                                                                            centroidDistanceFilter = True,
                                                                            mergeOverlapingDetectionsFilter = True,
                                                                            flagCloserToCenter=True)
            distance, x_distance, y_distance = getError(detection)

            if x_distance is not None and x_distance < 0:
                pos_atual_x += (abs(x_distance)/50)
                if pos_atual_x >= 180:
                    pos_atual_x = 180
            elif x_distance is not None and x_distance > 0:
                pos_atual_x -= (abs(x_distance)/50)
                if pos_atual_x <= 0:
                    pos_atual_x = 0
            if y_distance is not None and y_distance > 0:
                pos_atual_y += (abs(y_distance)/50)
                if pos_atual_y >= 130:
                    pos_atual_y = 130
            elif y_distance is not None and y_distance < 0:
                pos_atual_y -= (abs(y_distance)/50)
                if pos_atual_y <= 0:
                    pos_atual_y = 0
            sendValue(pos_atual_x,pos_atual_y)

            utilsDVS128.plotBoundingBox(displayEvents.gameDisplay, detection, displayEvents.m)

            t2 = time() - t
            displayEvents.printFPS(1/t2)
            pygame.display.update()
            pol, x, y, ts_LSB, ts_MSB = [], [], [], [], []

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
