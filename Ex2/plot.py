import serial
import matplotlib.pyplot as plt
from drawnow import *
import json

Xavalues = []
Yavalues = []
Zavalues = []

Xgvalues = []
Ygvalues = []
Zgvalues = []

Xvvalues = []
Yvvalues = []
Zvvalues = []

plt.ion()
cnt=0

serialBBB = serial.Serial('/dev/ttyUSB0', 115200)

def plotValues():
    plt.figure(1)
    plt.subplot(221)
    plt.title('Serial value from BBB\'s ACCEL')
    plt.grid(True)
    plt.ylabel('Values')
    plt.plot(Xavalues, 'r--', label='X')
    plt.plot(Yavalues, 'g--', label='Y')
    plt.plot(Zavalues, 'b--', label='Z')
    plt.legend(loc='upper right')
    plt.ylim(-10,10)

    plt.subplot(212)
    plt.title('Serial value from BBB\'s GYRO')
    plt.grid(True)
    plt.ylabel('Values')
    plt.plot(Xgvalues, 'r--', label='X')
    plt.plot(Ygvalues, 'g--', label='Y')
    plt.plot(Zgvalues, 'b--', label='Z')
    plt.legend(loc='upper right')
    plt.ylim(-200,200)

    plt.subplot(222)
    plt.title('Serial value from BBB\'s VELO')
    plt.grid(True)
    plt.ylabel('Values')
    plt.plot(Xvvalues, 'r--', label='X')
    plt.plot(Yvvalues, 'g--', label='Y')
    plt.plot(Zvvalues, 'b--', label='Z')
    plt.legend(loc='upper right')

#pre-load dummy data
for i in range(0,26):
    Xavalues.append(0)
    Yavalues.append(0)
    Zavalues.append(0)
    Xgvalues.append(0)
    Ygvalues.append(0)
    Zgvalues.append(0)
    Xvvalues.append(0)
    Yvvalues.append(0)
    Zvvalues.append(0)
    
while True:
    while (serialBBB.inWaiting()==0):
        pass
    valueRead = serialBBB.readline()
    
    try:
        pyDict = json.loads(valueRead)

        Xa = (float(pyDict['ACCEL']['x']))
        Ya = (float(pyDict['ACCEL']['y']))
        Za = (float(pyDict['ACCEL']['z']))
        Xg = (float(pyDict['GYRO']['x']))
        Yg = (float(pyDict['GYRO']['y']))
        Zg = (float(pyDict['GYRO']['z']))
        Xv = (float(pyDict['VELO']['x']))
        Yv = (float(pyDict['VELO']['y']))
        Zv = (float(pyDict['VELO']['z']))
        
        print('X - accel: ',Xa,' - gyro: ',Xg,' - velo: ',Xv)
        print('Y - accel: ',Ya,' - gyro: ',Yg,' - velo: ',Yv)
        print('Z - accel: ',Za,' - gyro: ',Zg,' - velo: ',Zv)
        print('\t----------')

        Xavalues.append(Xa)
        Yavalues.append(Ya)
        Zavalues.append(Za)
        Xgvalues.append(Xg)
        Ygvalues.append(Yg)
        Zgvalues.append(Zg)
        Xvvalues.append(Xv)
        Yvvalues.append(Yv)
        Zvvalues.append(Zv)
        Xavalues.pop(0)
        Yavalues.pop(0)
        Zavalues.pop(0)
        Xgvalues.pop(0)
        Ygvalues.pop(0)
        Zgvalues.pop(0)
        Xvvalues.pop(0)
        Yvvalues.pop(0)
        Zvvalues.pop(0)
        
        plt.show()
        drawnow(plotValues)

    except ValueError:
        print(valueRead)