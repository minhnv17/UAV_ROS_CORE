import rospy
import math
from uavlab411 import srv
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger

rospy.init_node("navigate_node")

uavpose = 0
is_pose = False
wps = [[3.0, 1.0, 1.0], [2.5, 2.0, 1.0], [2.0, 2.5, 1.0], 
[1.5, 3.0, 1.0], [1.0, 3.5, 1.0], [1.0, 4.0, 1.0], [1.0, 4.5, 1.0], [1, 5, 1]]
# wps = [[0, 1], [1, 1], [1, 2], [2, 2], [3, 4], [4, 4]]
get_telemetry = rospy.ServiceProxy('uavlab411/telemetry', srv.Telemetry)
navigate_to = rospy.ServiceProxy('uavlab411/navigate', srv.Navigate)
takeoff_srv = rospy.ServiceProxy('uavlab411/takeoff', srv.Takeoff)
land_srv = rospy.ServiceProxy('uavlab411/land', Trigger)

def navigate_wait(x, y, z, nav_mode, tolerance=0.15):
    res = navigate_to(x=x, y=y, z=z, nav_mode=nav_mode)
    if not res.success:
        return res

    time = rospy.get_rostime()
    while not rospy.is_shutdown():
        telemetry = get_telemetry("indoor")
        if(telemetry.mode != nav_mode):
            return res
        rospy.sleep(0.2)
        if (rospy.get_rostime() - time > rospy.Duration(20)):
            print("Can nav to wp!")
            return res


def takeoff(z):
    res = takeoff_srv(z=z)
    if not res.success:
        return res
    wait_for_telemetry()
    while not rospy.is_shutdown():
        telemetry = get_telemetry("indoor")
        if(telemetry.mode != 0):
            return res
        rospy.sleep(0.2)

def wait_for_telemetry():
    while not get_telemetry("indoor").isPose:
        rospy.sleep(0.5)

# takeoff
print("Take off now!")
takeoff(1.5)
rospy.sleep(2)

# navigate
for i in wps:
    wait_for_telemetry()
    print("navigate to wp " + str(i))
    navigate_wait(x=i[0], y=i[1], z=1, nav_mode=3,
                  tolerance=0.2)
    rospy.sleep(4)

land_srv()
