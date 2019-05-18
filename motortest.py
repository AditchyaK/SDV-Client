#code for the thrusters

import sys, time, Adafruit_PCA9685

try:
    pwm = Adafruit_PCA9685.PCA9685()
except:
    print ("Could not initialize PCA9685")
    sys.exit()
    
pwm.set_pwm(1, 1, int(1260))
#pwm.set_pwm(1, 1, int(1260))
#pwm.set_pwm(2, 1, int(1260))
#pwm.set_pwm(3, 1, int(1260))
#pwm.set_pwm(4, 1, int(1260))
#pwm.set_pwm(5, 1, int(1260))
print("Initializing motors...")
time.sleep(3)

print("Motor Forward test!")
pwm.set_pwm(1, 1, int(1400))
#pwm.set_pwm(1, 1, int(1400))
#pwm.set_pwm(2, 1, int(1400))
#pwm.set_pwm(3, 1, int(1400))
#pwm.set_pwm(4, 1, int(1400))
#pwm.set_pwm(5, 1, int(1400))
time.sleep(3)

"""
print("Motors are speeding up")
pwm.set_pwm(1, 1, int(1500))
time.sleep(3)
"""

print("Motors at rest")
pwm.set_pwm(1, 1, int(1260))
#pwm.set_pwm(1, 1, int(1260))
#pwm.set_pwm(2, 1, int(1260))
#pwm.set_pwm(3, 1, int(1260))
#pwm.set_pwm(4, 1, int(1260))
#pwm.set_pwm(5, 1, int(1260))
time.sleep(2)

print("Shutting down")
pwm.set_pwm(1, 0, int(0))
#pwm.set_pwm(1, 0, int(0))
#pwm.set_pwm(2, 0, int(0))
#pwm.set_pwm(3, 0, int(0))
#pwm.set_pwm(4, 0, int(0))
#pwm.set_pwm(5, 0, int(0))
sys.exit()

"""
#code for the actuator
import time 
from adafruit_motorkit import MotorKit

kit = MotorKit(address = 0x60)

kit.motor1.throttle = -1.0
time.sleep(2)
kit.motor1.throttle = 0

#code for stepper motor

from adafruit_motorkit import MotorKit
from adafruit_motor import stepper

kit = MotorKit()

while True:
    for i in range(200):
        kit.stepper2.onestep(style=stepper.SINGLE)

#code for the camera servo
import time
from adafruit_servokit import ServoKit
kit = ServoKit(channels=16)

#kit.servo[4].set_pulse_width_range(10, 2000)

kit.servo[4].angle = 180
time.sleep(1)
kit.servo[4].angle = 0
time.sleep(1)
"""
