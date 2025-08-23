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
navigate_global = rospy.ServiceProxy('uavlab411/navigate_global',srv.NavigateGlobal)
land_srv = rospy.ServiceProxy('uavlab411/land', Trigger)


gps_file = open("./gps_.txt","r")
content_list = gps_file.readlines()
path_cov =[]
gps_file.close()

for element in content_list:
    element = element.strip("\n")
    element = element.strip("],[")
    element = element.split(",")
    element[1]=element[1].strip()
    element[0] = float(element[0])
    element[1] = float(element[1])
    path_cov.append(element)

gps_file = open("./height_.txt","r")
content_list = gps_file.readlines()
gps_file.close()
height = []
for element in content_list:
    element = element.strip("\n")
    element = element.strip("],[")
    height.append(float(element))
print(height)
def navigate_wait(x, y, z, speed, tolerance=0.15):
    res = navigate_global(lat = x, lon =y, alt =z, speed = speed, tolerance = tolerance)
    if not res.success:
        return res

    time = rospy.get_rostime()
    while not rospy.is_shutdown():
        telemetry = get_telemetry("indoor")
        if(telemetry.mode == 2 ):
            return res
        rospy.sleep(0.2)
        if (rospy.get_rostime() - time > rospy.Duration(20)):
            print("Can nav to wp!")
            return res


def takeoff(z):
    res = takeoff_srv(z=z,tolerance = 0.1)
    if not res.success:
        return res
    # wait_for_telemetry()
    while not rospy.is_shutdown():
        telemetry = get_telemetry("indoor")
        if(telemetry.mode == 2):
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
for i in range(len(height)):
    print("navigate to wp " + str(i))
    navigate_wait(x=path_cov[i][0], y=path_cov[i][1], z=height[i], speed=1,
                  tolerance=0.2)

land_srv()