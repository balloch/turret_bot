import time
import math
import numpy as np
import Jetson.GPIO as GPIO


_GPIOCHIP_ROOT = "/dev/gpiochip0"

'''
### Notes ###
rough veritcal: servo(1)=102
angle can be used as get or set. output is float
angle commands only round down to the nearest 0.01radian 
   (but not perfect because 100.4deg should have but didn't round down to 100.3579deg)
resolution is 0.01 radians, or about 360/615th of an angle

printing angles 100-101 inclusive with 0.1deg increments yielded:
99.77226514843431
99.77226514843431
99.77226514843431
99.77226514843431
99.77226514843431
100.35786905246034
100.35786905246034
100.35786905246034
100.35786905246034
100.35786905246034
100.35786905246034
100.94347295648639

something is fundamentally off about this library: the servo is ~360deg, 
  but the library only allows commands from 0 to 180 and that covers ~135deg (middle is 90)
  top servo is from 50deg to 180deg. horizontal with this lib is 103.2858885725905


FOV of RPi V2: 62.2deg width, 48.8 deg height with 3280 x 2464 pixels
FOV of RPi V3 (75deg diag): 67.5deg width, 41.2 deg height with 4608x2592 pixels

https://learn.adafruit.com/adafruit-16-channel-servo-driver-with-raspberry-pi?view=all


gun trigger board is probably on standby, requires pin high for ~1sec to change to ready
  Red pin on board connection of fire select connector is low when in standby, high when active 
  IDK how but the pin at the switch is always high...something to do with the floating ground?
shot timing:
 * first round fires at 0.09s
 * second round timing at 0.28s
 * all rounds after that at increments of 0.1s

### END Notes ###
'''




GPIO.setmode(GPIO.BOARD)
trigger_pin = 11 
GPIO.setup(trigger_pin, GPIO.OUT)
## On startup send high once. For safety, until you can monitor whether the gun in in power saving mode or not maybe don't do this in deployment
GPIO.output(trigger_pin, GPIO.LOW)
#GPIO.output(trigger_pin, GPIO.HIGH)
#time.sleep(2)
GPIO.output(trigger_pin, GPIO.LOW)
time.sleep(2)
### TRIGGER TIMING TEST ###
#GPIO.output(trigger_pin, GPIO.HIGH)
#time.sleep(0.09)  # Expecting 1 shots
#GPIO.output(trigger_pin, GPIO.LOW)
#time.sleep(1)
#GPIO.output(trigger_pin, GPIO.HIGH)
#time.sleep(0.28)  # Expecting 2 shots
#GPIO.output(trigger_pin, GPIO.LOW)
#time.sleep(1)  
#GPIO.output(trigger_pin, GPIO.HIGH)
#time.sleep(0.38)  # Expecting 3 shots
#GPIO.output(trigger_pin, GPIO.LOW)
#time.sleep(1)
#GPIO.output(trigger_pin, GPIO.HIGH)
#time.sleep(0.48)  # Expecting 4 shots
#GPIO.output(trigger_pin, GPIO.LOW)
#time.sleep(1)
### TEST END ###

GPIO.cleanup()
