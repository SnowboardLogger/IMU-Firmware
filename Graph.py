import serial # Install pySerial or smth, not serial
from threading import Thread
import time
import re
import csv
import struct
import collections
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np


class serialPlot:
    def __init__(self, serialPort, serialBaud, plotLength):
        self.port = serialPort
        self.baud = serialBaud
        self.plotMaxLength = plotLength
        self.rawData = ""
        self.data = collections.deque([0] * plotLength, maxlen=plotLength)
        self.isRun = True
        self.isReceiving = False
        self.thread = None
        self.counter = 0
        self.plotTimer = 0
        self.previousTimer = 0
        # TODO: Write to CSV

        print('Attempting to connect to: ' + str(serialPort) + ' at ' + str(serialBaud) + ' BAUD')
        try:
            self.serialConnection = serial.Serial(serialPort, serialBaud, timeout=4)
            print('Connected to ' + str(serialPort) + ' at ' + str(serialBaud) + ' BAUD')
        except:
            print('Failed to connect with ' + str(serialPort) + ' at ' + str(serialBaud) + ' BAUD')
    
    def readSerialStart(self):
        if self.thread == None:
            self.thread = Thread(target=self.backgroundThread)
            self.thread.start()
            # Block till start receiving values
            while self.isReceiving != True:
                time.sleep(0.1)
    
    def getSerialData(self, frame, lines, lineValueText, lineLabel, timeText, ax):
        currentTimer = time.perf_counter()
        self.plotTimer = int((currentTimer - self.previousTimer) * 1000)
        self.previousTimer = currentTimer
        timeText.set_text('Plot interval = ' + str(self.plotTimer) + 'ms')
        value = self.rawData
        self.counter += 1
        self.data.append(value[0])    # we get the latest data point and append it to our array
        lines.set_data(range(self.counter, self.counter+self.plotMaxLength), self.data)
        ax.set_xlim(self.counter, self.plotMaxLength + self.counter)
        lineValueText.set_text(lineLabel + ' = ' + str(value[0]))
        # self.csvData.append(self.data[-1])
    
    def backgroundThread(self): # Second thread to retrieve data
        time.sleep(1.0)
        self.serialConnection.reset_input_buffer()
        while (self.isRun):
            self.rawData = re.findall('\d+.\d+', self.serialConnection.readline().decode())
            self.isReceiving = True
            print(self.rawData)

    def close(self):
        self.isRun = False
        self.thread.join()
        self.serialConnection.close()
        print('Disconnected from Port')
        # Find some way to send it to a csv

def plt_show():
    try:
        plt.show()
    except UnicodeDecodeError:
        plt_show()

def main():
    portName = 'COM100'
    baudRate = 115200
    maxPlotLength = 100
    s = serialPlot(portName, baudRate, maxPlotLength) # Initialization
    s.readSerialStart() # Starts secondary thread to read data

    # Plotting the data
    pltInterval = 50 # Period at which the plot animation updates [ms]
    xmin = 0
    xmax = maxPlotLength
    ymin = -(180)
    ymax = 180
    fig = plt.figure()
    ax = plt.axes(
        xlim=(xmin, xmax),
        ylim=(float(ymin - (ymax - ymin) / 10),
             float(ymax + (ymax - ymin) / 10)))
    ax.set_title('IMU Sensor Data')
    ax.set_xlabel('time')
    ax.set_ylabel('Pitch')

    lineLabel = 'Pitch Value'
    timeText = ax.text(0.50, 0.95, '', transform=ax.transAxes)
    lines = ax.plot([], [], label=lineLabel)[0]
    lineValueText = ax.text(0.50, 0.90, '', transform=ax.transAxes)
    anim = animation.FuncAnimation(fig, s.getSerialData, frames='save_count=MAX_FRAMES', fargs=(
            lines, lineValueText, lineLabel, timeText, ax), interval=pltInterval)
    plt.legend(loc="upper left")
    plt_show()

    s.close()

if __name__ == '__main__':
    main()
