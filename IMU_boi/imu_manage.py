#start of imu manage file
#!/usr/bin/env python3
"""
IMU_manage.py

Script to control donkey car with IMU navigation. Waypoints are set with GPS coordinates in degrees.

Call: ultra_gps_manage.py -drive
"""

# import GPS Planner and other DK parts
import donkeycar as dk
from gps_parts.ultrasonic import Ultrasonic
#from gps_parts.ultra_planner import Planner
from gps_parts.imu_planner import Planner
#from gps_parts.ultra_planner import Planner
from donkeycar.vehicle import Vehicle
from donkeycar.parts.actuator import PCA9685, PWMSteering, PWMThrottle

# from cv_parts.shape_test import CVCam
from gps_parts.imu import IMU

# other important modules
import serial
import pynmea2
import time
import threading


def drive(cfg):
    """
    drive(cfg)

    Add GPS, Planner, and actuator parts and call DK Vehicle.py to run car.
    @param: cfg - configuration file from dk calibration
            goalLocation - list of GPS coordinates in degrees
    @return: None
    """
    #if tORf == 1:
    #    print("I've come to talk with you again")

    file = 'line_data.txt'
    # initialize vehicle
    V = Vehicle()

    # Planner is a DK part that calculates control signals to actuators based on current location
    # from GPS
    planner = Planner(0, steer_gain=cfg.STEERING_P_GAIN,
                        throttle_gain=cfg.THROTTLE_P_GAIN)


    print("Because a vision softly creeping")
    # Actuators: steering and throttle
    steering_controller = PCA9685(cfg.STEERING_CHANNEL, busnum=1)
    steering = PWMSteering(controller=steering_controller,
                                    left_pulse=cfg.STEERING_LEFT_PWM,
                                    right_pulse=cfg.STEERING_RIGHT_PWM)

    throttle_controller = PCA9685(cfg.THROTTLE_CHANNEL, busnum=1)
    throttle = PWMThrottle(controller=throttle_controller,
                                    max_pulse=cfg.THROTTLE_FORWARD_PWM,
                                    zero_pulse=cfg.THROTTLE_STOPPED_PWM,
                                    min_pulse=cfg.THROTTLE_REVERSE_PWM)

    #Team2 addition
    #add OpenCV camera part. Outputs a stop command if shape is detected
    #cvcam = CVCam()
    #V.add(cvcam, outputs=['stop_cmd'], threaded=True)

    imu = IMU()
    V.add(imu, outputs=["heading"], threaded=False)


    # add planner, actuator parts
    V.add(planner, inputs=["heading", "desired_heading", "stop_cmd"], outputs=["steer_cmd", "throttle_cmd"])
    V.add(steering, inputs=['steer_cmd'])
    V.add(throttle, inputs=['throttle_cmd'])

    V.start()


if __name__ == '__main__':
    cfg = dk.load_config()  
    print("hello darkness my old friend")
    #file = 'line_data.txt'
    drive(cfg)
