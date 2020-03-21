"""
CAR CONFIG

This file is read by your car application's manage.py script to change the car
performance.

EXMAPLE
-----------
import dk
cfg = dk.load_config(config_path='~/d2/config.py')
print(cfg.CAMERA_RESOLUTION)

"""


import os

#PATHS
CAR_PATH = PACKAGE_PATH = os.path.dirname(os.path.realpath(__file__))
DATA_PATH = os.path.join(CAR_PATH, 'data')
MODELS_PATH = os.path.join(CAR_PATH, 'models')

#VEHICLE
DRIVE_LOOP_HZ = 20
MAX_LOOPS = 100000

#CAMERA
#IMAGE_W = 160
#IMAGE_H = 120
#IMAGE_ASPECT_RATIO = 100
#CAMERA_FRAMERATE = DRIVE_LOOP_HZ

#STEERING
STEERING_CHANNEL = 1            #channel on the 9685 pwm board 0-15
STEERING_LEFT_PWM = 460         #pwm value for full left steering
STEERING_RIGHT_PWM = 290        #pwm value for full right steering

#THROTTLE
THROTTLE_CHANNEL = 2            #channel on the 9685 pwm board 0-15
THROTTLE_FORWARD_PWM = 500      #pwm value for max forward throttle
THROTTLE_STOPPED_PWM = 370      #pwm value for no movement
THROTTLE_REVERSE_PWM = 220      #pwm value for max reverse throttle

# Region of interst cropping
# only supported in Categorical and Linear models.
# If these crops values are too large, they will cause the stride values to become negative and the model with not be valid.
ROI_CROP_TOP = 0                    #the number of rows of pixels to ignore on the top of the image
ROI_CROP_BOTTOM = 0                 #the number of rows of pixels to ignore on the bottom of the image

################ GPS PROJECT ###################
#CONTROLLER
THROTTLE_P_GAIN = 0.45
STEERING_P_GAIN = 0.4

#GPS SERIAL
PORT = '/dev/ttyACM0'
BAUD_RATE = 9600
TIMEOUT =  1  # seconds
################ GPS PROJECT ###################

#TRAINING
BATCH_SIZE = 128
TRAIN_TEST_SPLIT = 0.8


#JOYSTICK
USE_JOYSTICK_AS_DEFAULT = True
JOYSTICK_MAX_THROTTLE = 0.50
JOYSTICK_STEERING_SCALE = 0.85
AUTO_RECORD_ON_THROTTLE = True

#RNN or 3D
SEQUENCE_LENGTH = 3
IMAGE_DEPTH = 3

#IMU
HAVE_IMU = False

#BEHAVIORS
TRAIN_BEHAVIORS = False
BEHAVIOR_LIST = ['Left_Lane', "Right_Lane"]
BEHAVIOR_LED_COLORS =[ (0, 10, 0), (10, 0, 0) ] #RGB tuples 0-100 per chanel
