# Simple demo of of the PCA9685 PWM servo/LED controller library.
# This will move channel 0 from min to max position repeatedly.
# Author: Tony DiCola
# License: Public Domain
from __future__ import division
import string
import time
import rospy
from uavlab411.msg import control_robot_msg

# Import the PCA9685 module.
import Adafruit_PCA9685


# Uncomment to enable debug output.
#import logging
#logging.basicConfig(level=logging.DEBUG)

# Initialise the PCA9685 using the default address (0x40).
pwm = Adafruit_PCA9685.PCA9685()

# Alternatively specify a different address and/or bus:
#pwm = Adafruit_PCA9685.PCA9685(address=0x41, busnum=2)

# Helper function to make setting a servo pulse width simpler.
def set_servo_pulse(channel, pulse):
    pulse_length = 1000000    # 1,000,000 us per second
    pulse_length //= 60       # 60 Hz
    print('{0}us per period'.format(pulse_length))
    pulse_length //= 4096     # 12 bits of resolution
    print('{0}us per bit'.format(pulse_length))
    pulse *= 1000
    pulse //= pulse_length
    print(pulse)
    pwm.set_pwm(channel, 0, pulse)

# Set frequency to 60hz, good for servos.
pwm.set_pwm_freq(60)
stamp = 0
def callback(control_robot):
    Pstep2 = 400 - control_robot.step2 * (400 - 230) / 90
    Pstep3 = 130 + (180 - control_robot.step3) * (600 - 130) / 180
    Pstep4 = 350 + (90 - control_robot.step4) * (550 - 350) / 90
    Pstep5 = 400 - control_robot.step5 * 200
    global stamp
    stamp = control_robot.header.stamp.secs

    pwm.set_pwm(0, 0, int(Pstep2))
    pwm.set_pwm(1, 0, int(Pstep3))
    pwm.set_pwm(2, 0, int(Pstep4))
    pwm.set_pwm(3, 0, int(Pstep5))
    
    rospy.loginfo("Step 2 Pulse: %i", control_robot.step2)
    rospy.loginfo("Step 3 Pulse: %i", control_robot.step3)
    rospy.loginfo("Step 4 Pulse: %i", control_robot.step4)
    rospy.loginfo("Step 5 Pulse: %i", control_robot.step5)

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('uavlab411/control_robot', control_robot_msg, callback)

if __name__ == '__main__':
    listener()
    while not rospy.is_shutdown:
        timeOut = rospy.Time.now().secs - stamp
        print("Time Out: ", timeOut)
        if timeOut >= 3:
            pwm = Adafruit_PCA9685.PCA9685()
    rospy.spin()
    

