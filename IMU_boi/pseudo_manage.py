#!/usr/bin/env python3
"""
pseudo_manage.py

Script to control donkey car with pseudo navigation. Waypoints are set with GPS coordinates in degrees.

Call: ultra_gps_manage.py -drive
"""

# import GPS Planner and other DK parts
import donkeycar as dk
from gps_parts.gps import GPS
from gps_parts.ultrasonic import Ultrasonic
from gps_parts.pseudo_planner import Planner
from donkeycar.vehicle import Vehicle
from donkeycar.parts.actuator import PCA9685, PWMSteering, PWMThrottle

# from cv_parts.shape_test import CVCam
from gps_parts.imu import IMU
from gps_parts.trajectory import Trajectory


# other important modules
import serial
import pynmea2
import time
import threading


def drive(cfg):
    """
    drive(cfg, goalLocation)

    Add GPS, Planner, and actuator parts and call DK Vehicle.py to run car.
    @param: cfg - configuration file from dk calibration
            goalLocation - list of GPS coordinates in degrees
    @return: None
    """
    # initialize vehicle
    V = Vehicle()

    # GPS is a DK part that will poll GPS data from serial port
    # and output current location in radians.
    #gps = GPS(cfg.BAUD_RATE, cfg.PORT, cfg.TIMEOUT)

    # IMU addition
    imu = IMU()

    # Planner is a DK part that calculates control signals to actuators based on current location
    # from GPS
    planner = Planner(steer_gain=cfg.STEERING_P_GAIN,
                        throttle_gain=cfg.THROTTLE_P_GAIN)

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

    # add threaded part for gps controller
    #V.add(gps, outputs=["currLocation", "prevLocation"], threaded=True)

    #TODO replace with CvCam code
    ultrasonic = Ultrasonic()
    V.add(ultrasonic, outputs=['stop_cmd'], threaded=True)

    #Team2 addition
    #add OpenCV camera part. Outputs a stop command if shape is detected
    #cvcam = CVCam()
    #V.add(cvcam, outputs=['stop_cmd'], threaded=True)

    V.add(imu, outputs=['heading'], threaded=True)


    #desired heading part
    d_head = Trajectory()
    V.add(d_head, outputs=['desired_heading'], threaded=True)



    # add planner, actuator parts
    V.add(planner, inputs=['heading', 'desired_heading', 'stop_cmd'], outputs=["steer_cmd", "throttle_cmd"])
    V.add(steering, inputs=['steer_cmd'])
    V.add(throttle, inputs=['throttle_cmd'])

    V.start()


if __name__ == '__main__':
    cfg = dk.load_config()  
    drive(cfg)
