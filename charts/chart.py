import serial
import time
import numpy as np
import matplotlib.pyplot as plt
import msvcrt as msv

# set buffer
from collections import deque
BUFFER_SIZE = 200
buff_Pitch_GYRO = deque(BUFFER_SIZE*[0],BUFFER_SIZE)
buff_Pitch_ACC = deque(BUFFER_SIZE*[0],BUFFER_SIZE)
buff_KalmanAngleX = deque(BUFFER_SIZE*[0],BUFFER_SIZE)
t = np.linspace(0,2,200)

Pitch_GYRO = 0 
Pitch_ACC = 0
KalmanAngleX = 0 

DEVICE = "COM3"
BAUDRATE = 115200

com = serial.Serial(DEVICE, BAUDRATE, timeout=5)
time.sleep(2)

plt.ylim(-90,90)
line_Pitch_GYRO, = plt.plot(t,buff_Pitch_GYRO)
line_Pitch_ACC, = plt.plot(t,buff_Pitch_ACC)
line_KalmanAngleX, = plt.plot(t,buff_KalmanAngleX)
plt.ion()
plt.show()
s = com.readline().strip()
while 1:
    s = com.readline().strip()
    s = s.decode()
    p = s.split(':')

    #print(p)
    if p[0] == 'Pitch_GYRO':
        Pitch_GYRO = -float(p[1])
        buff_Pitch_GYRO.appendleft(Pitch_GYRO)
        line_Pitch_GYRO.set_ydata(buff_Pitch_GYRO)
        #time.sleep(0.01)
        plt.pause(0.01)
        print("Pitch_GYRO : ",Pitch_GYRO)
    if p[6] == 'Pitch_ACC':
        Pitch_ACC = -float(p[7])
        buff_Pitch_ACC.appendleft(Pitch_ACC)
        line_Pitch_ACC.set_ydata(buff_Pitch_ACC)
        #time.sleep(0.01)
        plt.pause(0.01)
        print("Pitch_ACC : ",Pitch_ACC)
    if p[10] == 'KalmanAngleX':
        KalmanAngleX = -float(p[11])
        buff_KalmanAngleX.appendleft(KalmanAngleX)
        line_KalmanAngleX.set_ydata(buff_KalmanAngleX)
        #time.sleep(0.01)
        plt.pause(0.01)
        print("KalmanAngleX : ",KalmanAngleX)

com.close()