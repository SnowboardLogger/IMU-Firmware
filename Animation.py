from vpython import *
from time import *
import numpy as np
import math
import serial

data =serial.Serial('com100',115200)
sleep(1)
 
scene.range=5
toRad=2*np.pi/360
toDeg=1/toRad
scene.forward=vector(-1,-1,-1)
 
scene.width=600
scene.height=600
 
xarrow = arrow(lenght=2, shaftwidth=.1, color=color.red,axis=vector(1,0,0))
yarrow = arrow(lenght=2, shaftwidth=.1, color=color.green,axis=vector(0,1,0))
zarrow = arrow(lenght=4, shaftwidth=.1, color=color.blue,axis=vector(0,0,1))
 
frontArrow = arrow(length=4,shaftwidth=.1,color=color.purple,axis=vector(1,0,0))
upArrow = arrow(length=1,shaftwidth=.1,color=color.magenta,axis=vector(0,1,0))
sideArrow = arrow(length=2,shaftwidth=.1,color=color.orange,axis=vector(0,0,1))

IMU=box(lenght=1.75,width=.6,height=.1,color=color.green)

while (True):
    while (data.inWaiting()==0):
        pass
    dataPacket = data.readline()
    dataPacket = str(dataPacket,'utf-8')
    splitPacket = dataPacket.split(" ")
    roll = float(splitPacket[0])*toRad
    pitch = float(splitPacket[1])*toRad
    yaw = float(splitPacket[2])*toRad + np.pi
    aCal = float(splitPacket[3])
    gCal = float(splitPacket[4])
    mCal = float(splitPacket[5])
    sCal = float(splitPacket[6])
    print("Roll: ", round(roll*toDeg, 2)," Pitch: ", round(pitch*toDeg, 2),
          "Yaw: ", round(yaw*toDeg, 2),
        "aCal: ", aCal, " gCal: ", gCal,
        "mCal: ", mCal, " sCal: ", sCal
    )
    rate(50)
    k = vector(cos(yaw)*cos(pitch), sin(pitch),sin(yaw)*cos(pitch))
    y = vector(0,1,0)
    s = cross(k,y)
    v = cross(s,k)
    vrot = v * cos(roll) + cross(k, v) * sin(roll)
 
    frontArrow.axis = k
    sideArrow.axis = s
    upArrow.axis = vrot
    IMU.axis = cross(k, vrot)
    IMU.up = vrot
    sideArrow.length = 2
    frontArrow.length = 4
    upArrow.length = 1
