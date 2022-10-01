import serial
import time
import numpy as np
import matplotlib.pyplot as plt

DEVICE = '/dev/rfcomm1'
BAUD_RATE = 9600

# Connect to the device
s = serial.Serial(DEVICE, BAUD_RATE)
print('Connect to', DEVICE)
ax = 0
ay = 0
az = 0

my = 0
mx = 0
mz = 0

gx = 0
gy = 0
gz = 0

hard_iron_bias_x =  113.64034035344433
hard_iron_bias_y =  -37.23524005238167
hard_iron_bias_z =  -9.388401954577757

# set buffer
from collections import deque
BUFFER_SIZE = 200
buff_ax = deque(BUFFER_SIZE*[0],BUFFER_SIZE)
buff_ay = deque(BUFFER_SIZE*[0],BUFFER_SIZE)
buff_az = deque(BUFFER_SIZE*[0],BUFFER_SIZE)
buff_mx = deque(BUFFER_SIZE*[0],BUFFER_SIZE)
buff_my = deque(BUFFER_SIZE*[0],BUFFER_SIZE)
buff_mz = deque(BUFFER_SIZE*[0],BUFFER_SIZE)
buff_gx = deque(BUFFER_SIZE*[0],BUFFER_SIZE)
buff_gy = deque(BUFFER_SIZE*[0],BUFFER_SIZE)
buff_gz = deque(BUFFER_SIZE*[0],BUFFER_SIZE)
t = np.linspace(0,2,200)


def get_raw_data(s):
    global ax,ay,az,gx,gy,gz,mx,my,mz
    params = s.split(',')
    #print(params)
    for p in params:
        try:
            key,value = p.split(':')
            if key == 'AX':
                ax = float(value)
            if key == 'AY':
                ay = float(value)
            if key == 'AZ':
                az = float(value)

            if key == 'MX':
                mx = float(value) - hard_iron_bias_x
                mx = mx * 0.92
            if key == 'MY':
                my = float(value) - hard_iron_bias_y
                my = my * 0.92
            if key == 'MZ':
                mz = float(value) - hard_iron_bias_z
                mz = mz * 0.92

            if key == 'GX':
                gx = float(value)
            if key == 'GY':
                gy = float(value)
            if key == 'GZ':
                gz = float(value)
        except:
            print("data received with error")
    
    return ax,ay,az,gx,gy,gz,mx,my,mz
    



buff_mag = []
def collect_mag_data(data,n=250):
    global buff_mag
    if len(buff_mag) < n:
        buff_mag.append(data)
    if len(buff_mag) == n:
        with open("mag_out_sample.txt","+w") as f:
            for mag_data in buff_mag:
                f.write(mag_data+"\r\n")

plt.ylim(-10,10)
line_ax, = plt.plot(t,buff_ax)
line_ay, = plt.plot(t,buff_ay)
line_az, = plt.plot(t,buff_az)
plt.ion()
#plt.show()

allan_ax = []

while 1:
    data = s.readline()

    print(data)
    print(data.decode().strip())
    data = data.decode().strip()

    get_raw_data(data)
    buff_ax.appendleft(ax)
    buff_ay.appendleft(ay)
    buff_az.appendleft(az)
    buff_gx.appendleft(gx)
    buff_gy.appendleft(gy)
    buff_gz.appendleft(gz)
    line_ax.set_ydata(buff_ax)
    line_ay.set_ydata(buff_ay)
    line_az.set_ydata(buff_az)
    allan_ax.append(ax)
    if len(allan_ax) == 200:
        a = np.array(allan_ax)
        np.savetxt("data.txt",a)

    #time.sleep(0.01)
    #plt.pause(0.0001)
    
    #collect_mag_data("{},{},{}".format(mx,my,mz))
