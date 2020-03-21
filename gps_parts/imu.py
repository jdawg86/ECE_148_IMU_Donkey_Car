#create IMU for bno080
"""
imu.py

IMU part for Donkeycar framework.

@author: Jacinth Gudetti
"""

import serial
from threading import Thread
import time

class IMU:
    '''
    IMU Sensor
    '''
    def __init__(self,port='/dev/ttyACM0'):

        self.ser = serial.Serial(port, 115200)
        self.head = 0
        self.on = True

    #blocking single-run version
    def read(self):
        #get the two distances from the arduino via serial
        #distances are in centimeters
        try:
            line = self.ser.readline().strip()
            self.head = float(line)
        except:
            print("OH NO")
            pass

    #call read() with run()

    run = read

    #def run(self):
        #return self.update()



    #this gets called after an update
    #blindly return the values
    def run_threaded(self):
        return self.head

    #independent thread version
    def update(self):
        while self.on:
            #get the heading from the arduino via serial
            #heading is in radians   
           
            self.read()
            self.head = self.ser.readline().strip()
            self.head = float(self.head)
            '''
            threads = []
            process = Thread(target = imu.read())
            process.start()
            threads.append(process)
            for process in threads:
                process.join()
            '''




'''
if __name__=="__main__":
    imu = IMU()
    threads = []
    for i in range(20):
        print("heading: {}".format(imu.poll())
        process = threading.Thread(target = imu.poll())
        process.start()
        threads.append(process)
    for process in threads:
        process.join()
        print("heading: {}".format(str(imu.poll())))
'''
