import serial
import time
import numpy as np
import matplotlib.pyplot as plt
import msvcrt as msv

# set buffer
from collections import deque
BUFFER_SIZE = 200
buff_ax = deque(BUFFER_SIZE*[0],BUFFER_SIZE)
t = np.linspace(0,2,200)
AngleX = 0

DEVICE = "COM4"
BAUDRATE = 9600

com = serial.Serial(DEVICE, BAUDRATE, timeout=5)
time.sleep(2)

plt.ylim(-90,90)
line_ax, = plt.plot(t,buff_ax)
plt.ion()
plt.show()

while 1:
    s = com.readline().strip()
    s = s.decode()
    p = s.split(':')
    if p[0] == '\x00KalmanAngleX ':
        AngleX = -float(p[1])
        buff_ax.appendleft(AngleX)
        line_ax.set_ydata(buff_ax)
        time.sleep(0.01)
        plt.pause(0.0001)
        #print(AngleX)
    

com.close()