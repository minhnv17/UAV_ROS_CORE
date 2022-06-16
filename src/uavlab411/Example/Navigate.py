import rospy
import math
from uavlab411 import srv
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger

rospy.init_node("navigate_node")

uavpose = 0
is_pose = False
wps = [[0, 1], [1, 1], [1, 2], [2, 2], [3, 4], [4, 4]]
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
        if math.sqrt((telemetry.x - x)**2 + (telemetry.y - y)**2 + (telemetry.z - z)**2) < tolerance:
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
        if abs(telemetry.z - z) > 0.1:
            return res
        rospy.sleep(0.2)

def wait_for_telemetry():
    while not is_pose:
        rospy.sleep(0.5)

def uavpose_cb(msg):
    global uavpose, is_pose
    is_pose = True
    uavpose = msg

rospy.Subscriber('uavlab411/uavpose', PoseStamped, uavpose_cb)

# takeoff
print("Take off now!")
takeoff(1.5)
rospy.sleep(10)

# navigate
for i in wps:
    wait_for_telemetry()
    print("navigate to wp " + str(i))
    navigate_wait(x=i[0], y=i[1], z=1, nav_mode=3,
                  tolerance=0.2)
    rospy.sleep(4)

land_srv()
rospy.spin()