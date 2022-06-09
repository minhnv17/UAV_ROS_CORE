import rospy
import math
from uavlab411 import srv
from geometry_msgs.msg import PoseStamped

rospy.init_node("navigate_node")

uavpose = 0
is_pose = False

takeoff_srv = rospy.ServiceProxy('uavlab411/takeoff', srv.Takeoff)

def takeoff(z):
    res = takeoff_srv(z=z)
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

def uavpose_cb(msg):
    global uavpose, is_pose
    is_pose = True
    uavpose = msg
        
rospy.Subscriber('uavlab411/uavpose', PoseStamped, uavpose_cb)

# takeoff
print("Take off now!")
takeoff(1.5)
rospy.sleep(3)

rospy.spin()
