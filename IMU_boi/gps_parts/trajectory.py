"""
trajectory.py


@author: Jacinth Gudetti, Liam Engel, Tanner Hansen
"""

import serial

#send a stop signal when something is less than 50cm away

class Trajectory:
    '''
    Ultrasonic sensor
    '''
    def __init__(self):

        self.traj = 0.8
        self.on = True

    #blocking single-run version
    def run(self):
        self.traj = 0.8
        return self.traj

    #this gets called after an update
    #blindly return the values
    def run_threaded(self):
        #returns stop_cmd
        return self.traj

    #independent thread version
    def update(self):
        while self.on:
            self.traj = float(self.traj)

    def shutdown(self):
        self.on = False
        return
        
def _main():
    
    traj = Trajectory()

    try:
        while(True):
            traj = traj.run()
            
    except(KeyboardInterrupt):
        print("Exiting...")

if(__name__ == '__main__'):
    _main()
