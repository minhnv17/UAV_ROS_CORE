import rospy
import math
from uavlab411 import srv
from geometry_msgs.msg import PoseStamped

rospy.init_node("navigate_node")

uavpose = 0
is_pose = False
wps = [[1, 1], [1, 1.5], [1, 2], [2, 2.3], [3, 4]]
navigate_to = rospy.ServiceProxy('uavnavigate', srv.Navigate)


def navigate_wait(x, y, z, nav_mode, tolerance=0.2, auto_arm=False):
    res = navigate_to(x=x, y=y, z=z, nav_mode=nav_mode, auto_arm=auto_arm)

    if not res.success:
        return res

    time = rospy.get_rostime()
    while not rospy.is_shutdown():
        if math.sqrt((uavpose.pose.position.x - x)**2 + (uavpose.pose.position.y - y)**2) < tolerance:
            return res
        rospy.sleep(0.2)
        if (rospy.get_rostime() - time > rospy.Duration(10)):
            print("Can nav to wp!")
            return res


def takeoff(z):
    res = navigate_to(z=1, auto_arm=True)
    if not res.success:
        return res
    wait_for_telemetry()
    while not rospy.is_shutdown():
        if abs(uavpose.pose.position.z - z) > 0.1:
            return res
        rospy.sleep(0.2)


def wait_for_telemetry():
    while not is_pose:
        rospy.sleep(0.5)
        print("no local position!")


def uavpose_cb(msg):
    global uavpose, is_pose
    is_pose = True
    uavpose = msg


rospy.Subscriber('uavlab411/uavpose', PoseStamped, uavpose_cb)

# takeoff
print("Take off now!")
takeoff(1.5)
rospy.sleep(3)

# navigate
for i in wps:
    wait_for_telemetry()

    print("navigate to wp " + str(i))
    navigate_wait(x=i[0], y=i[1], z=1, nav_mode=3,
                  tolerance=0.2, auto_arm=False)

rospy.spin()
