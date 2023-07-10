import time
import math
import Jetson.GPIO as GPIO
#GPIO.setmode(GPIO.BOARD)
from adafruit_servokit import ServoKit
import numpy as np


_GPIOCHIP_ROOT = "/dev/gpiochip0"


### Notes ###
# rough veritcal: servo(1)=102
# angle can be used as get or set. output is float
# angle commands only round down to the nearest 0.01radian 
#   (but not perfect because 100.4deg should have but didn't round down to 100.3579deg)
# resolution is 0.01 radians, or about 360/615th of an angle

# printing angles 100-101 inclusive with 0.1deg increments yielded:
#99.77226514843431
#99.77226514843431
#99.77226514843431
#99.77226514843431
#99.77226514843431
#100.35786905246034
#100.35786905246034
#100.35786905246034
#100.35786905246034
#100.35786905246034
#100.35786905246034
#100.94347295648639

# something is fundamentally off about this library: the servo is ~360deg, 
# but the library only allows commands from 0 to 180 and that covers ~135deg (middle is 90)
# top servo is from 50deg to 180deg. horizontal with this lib is 103.2858885725905


#FOV of RPi V2: 62.2deg width, 48.8 deg height with 3280 x 2464 pixels
#FOV of RPi V3 (75deg diag): 67.5deg width, 41.2 deg height with 4608x2592 pixels

#https://learn.adafruit.com/adafruit-16-channel-servo-driver-with-raspberry-pi?view=all

### END Notes ###



### SETUP ###
kit = ServoKit(channels=16)
#kit.servo[0].actuation_range = 160   ## only works for making it lower than 180
kit.servo[0].set_pulse_width_range(1000, 2000)  ## tune this for range
mode = GPIO.getmode()

print(mode)
print(GPIO.BOARD)
print(GPIO.BCM)
print(GPIO.CVM)
print(GPIO.TEGRA_SOC)
print(GPIO.BOARD)
#trigger_pin = 13  # For GPIO.BOARD (Nano breakout pin)
led_pin = "GPIO_PE6"#"GPIO_PZ0"  # For GPIO.TEGRA_SOC, servokit default
#trigger_pin = "D13"  # For digitalio with blinka 

GPIO.setup(led_pin, GPIO.OUT)
### END SETUP ###


### INIT AND CALIB ###
## On startup send high once. For safety, until you can monitor whether the gun in in power saving mode or not maybe don't do this in deployment
GPIO.output(led_pin, GPIO.LOW)
GPIO.output(led_pin, GPIO.HIGH)
time.sleep(2)
GPIO.output(led_pin, GPIO.LOW)

##Servo move to min 
kit.servo[0].angle = 0
kit.servo[1].angle = 50
time.sleep(1)

#kit.servo[1].angle=55
#print(kit.servo[1].angle)

## Servo Initialize
servo_angle_init = np.array([90,103.2858885725905])
print('Init: ')
kit.servo[0].angle = servo_angle_init[0]
kit.servo[1].angle = servo_angle_init[1]
time.sleep(1)
current_angle = np.array([kit.servo[0].angle,kit.servo[1].angle])
print('Angles: ', current_angle)
init_diff = current_angle - servo_angle_init
print('Init Diff: ', init_diff)

### END INIT AND CALIB ###


### Object tracking simulator
## From camera
img_width = 1280
img_height = 720
img_center = np.array([img_height/2,img_width/2])  #row,col  # needs to be augmented by aim calibration
fov_width = 62.2  #degrees

z_dist_pixels = 0.5*img_width/math.tan(fov_width/2*2*math.pi/360)  # 1060.94pix


## Fake data
stuff= True
bb_size = [36,32]
num_samples=20
data_bb_min = np.linspace(
        start=[0,0],
        stop=[img_height,img_width], 
        num=num_samples, 
        endpoint=False, 
        dtype=int)
data_bb_max = data_bb_min + bb_size
#data_bb = np.concatenate((data_bb_min, data_bb_max), axis=1)
data_bb = np.stack((data_bb_min, data_bb_max))
data_centroids = data_bb.mean(axis=0) 
data_vel = data_bb_min-np.append(data_bb_min[1:,:],data_bb_min[-1,:]).reshape(num_samples,-1)

# Generate desired angles
normalized = (data_centroids - img_center)/z_dist_pixels
angles = np.arctan(normalized)
degrees = angles/(2*np.pi)*360



# Iterate through these angles
for step in degrees:
    desired_angle = [step[1] + servo_angle_init[0], step[0] + servo_angle_init[1]]
    print('Desired Angle: ',  desired_angle) 
    kit.servo[0].angle = desired_angle[0]
    kit.servo[1].angle = desired_angle[1]
    time.sleep(1)
    print('final Angle: ', kit.servo[0].angle, kit.servo[1].angle)

"""
while(stuff):
    ## The desired location should be slightly ahead of the bb middle in the direction of the vel
    # From object detector
    obj_bb = np.array([[16,16],[64,48]]) #np.array([[0,2],[0,2]]) #[[min_row,min_col],[max_row, max_col]]
    obj_vel = np.array([0,0])  #[row positive down,col positive right]

    #calculated
    bb_center = obj_bb.mean(axis=0)
    target_center = bb_center-img_center    
    obj_area = np.prod(np.diff(obj_bb,axis=0))
    if obj_area>1:

        target_bias = [0,0]
    else:
        target_bias = [0,0]
    target = target_center + target_bias

    ## Calculate relative angle, given top left [0,0] pixel and the FOV in notes

    pan_angle = math.arctan(target[0],z_dist_pixels)
    tilt_angle = math.arctan(target[1],z_dist_pixels)
"""

GPIO.cleanup()
