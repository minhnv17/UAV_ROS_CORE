# import cv2
import numpy as np
import math


from __future__ import division
import string
import time
import rospy
from sensor_msgs import Range
import Adafruit_PCA9685

pwm = Adafruit_PCA9685.PCA9685()


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


def set_servo_angle(step1, step2, step3, step4, step5):
    Pstep1 = 540 - 440 / 180 * step1
    Pstep2 = 100 + 450 / 180 * step2
    Pstep3 = 130 + 470 / 180 * step3
    Pstep4 = 150 + 450 / 180 * step4
    Pstep5 = 400 - step5*200
    pwm.set_pwm(0, 0, int(Pstep1))
    pwm.set_pwm(1, 0, int(Pstep2))
    pwm.set_pwm(2, 0, int(Pstep3))
    pwm.set_pwm(3, 0, int(Pstep4))
    pwm.set_pwm(4, 0, int(Pstep5))

# Set frequency to 60hz, good for servos.
pwm.set_pwm_freq(60)

def callback(msg_range):
    find_angle((320,int(msg_range.range)))
def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('/range_finder/range', Range, callback)




L1 = 120
L2 = 120
L3 = 130
p1 = (0, 20)
p2 = (0, 0)
p3 = (0, 0)
a1 = 90
a2 = 160
a3 = 120

# space = np.zeros((500, 1000, 3), dtype=np.uint8)
# space.fill(255)

def find_angle(des):
    global a1, a2, a3, p1, p2, p3, L1, L2, L3
    d = math.dist(p1, des)
    a1_min = a1_max = 0
    # dieu kien de goc a3 tu
    cos_a1_min = (L1**2 + d**2 - L2**2 - L3**2)/(2*L1*d)
    # dieu kien de co tam giac p2 p3 des
    cos_a1_max = (L1**2 + d**2 - (L2+L3)**2) / (2*L1*d)
    # print(cos_a1_max_1, cos_a1_min, cos_a1_max)
    if cos_a1_min < 1:
        a1_min = math.acos(cos_a1_min)
    if cos_a1_max < 1:
        a1_max = math.acos(cos_a1_max)
    # print((a1_max + math.atan2(des[1], des[0]))/math.pi*180)
    a1_min = (a1_min + math.atan2(des[1]-p1[1], des[0]-p1[0]))
    a1_max = (a1_max + math.atan2(des[1]-p1[1], des[0]-p1[0]))

    # space.fill(255)
    # cv2.circle(space, des, 10, (255, 0, 0), -1)
    # print(a1_max/math.pi*180, a1_min/math.pi*180)
    while True:
        a1 = (a1_max + a1_min)/2
        # a1 = a1_max
        p2 = (p1[0]+int(L1 * math.cos(a1)),
              p1[1]+int(L1 * math.sin(a1)))
        y = des[0] - p2[0]
        x = des[1] - p2[1]
        # print(x, y)
        a3 = math.acos((x**2 + y**2 - L2**2 - L3**2)/(2*L2*L3))
        a2 = math.atan2(y, x) - math.atan2(L3*math.sin(a3), L2+L3*math.cos(a3))
        # print(a3/math.pi*180, a2/math.pi*180)
        p3 = (p2[0] + int(L2*math.sin(a2)), p2[1] + int(L2*math.cos(a2)))
        if math.dist(p3, p1)**2 < L1**2 + L2**2:
            a1_max -= 0.1
            continue
        des1 = (p3[0] + int(L3*math.sin(a2+a3)),
                p3[1] + int(L3*math.cos(a2+a3)))
        # cv2.line(space, p1, p2, (0, 255, 0), 5)
        # cv2.line(space, p2, p3, (0, 255, 0), 5)
        # cv2.line(space, p3, des1, (0, 255, 0), 5)
        # cv2.circle(space, p1, 10, (0, 0, 255), -1)
        # cv2.circle(space, p2, 10, (0, 0, 255), -1)
        # cv2.circle(space, p3, 10, (0, 0, 255), -1)
        # cv2.imshow("res", space)
        A1 = 180 - a1/math.pi * 180
        A2 = 180 - A1 + a2/math.pi * 180
        A3 = 90 + a3/math.pi*180
        print(A1, A2, A3)
        break

    # cv2.waitKey(0)
# find_angle((32, 18))
if __name__ == '__main__':
    listener()
    while not rospy.is_shutdown:
        timeOut = rospy.Time.now().secs - stamp
        print("Time Out: ", timeOut)
        if timeOut >= 3:
            pwm = Adafruit_PCA9685.PCA9685()
    rospy.spin()