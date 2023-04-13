import serial
import time

data = serial.Serial('com117', 115200)
time.sleep(1)
while(True):
    while(data.inWaiting() == 0):
        pass
    dataPacket = data.readline()
    dataPacket = str(dataPacket, 'utf-8')
    splitPacket = dataPacket.split(' ')
    Acal = float(splitPacket[0])
    Gcal = float(splitPacket[1])
    Mcal = float(splitPacket[2])
    Scal = float(splitPacket[3])
    Pitch = float(splitPacket[4])
    Roll = float(splitPacket[5])
    Yaw = float(splitPacket[6])
    print("XAcc=", Acal, " YAcc=", Gcal, " ZAcc=", Mcal,
        " Scal=", Scal, " Pitch=", Pitch, " Roll=", Roll, " Yaw=",Yaw)
