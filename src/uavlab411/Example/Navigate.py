import rospy
from uavlab411 import srv
from geometry_msgs.msg import PoseStamped

rospy.init_node("navigate_node")


navigate_to = rospy.ServiceProxy('uavnavigate', srv.Navigate)

def uavpose_cb(msg):
    print(msg)

rospy.Subscriber('uavlab411/uavpose', PoseStamped, uavpose_cb)

navigate_to(x = 2, y = 3, z = 1.5, auto_arm = False)

rospy.spin()


# res = navigate_to(z = 1, auto_arm = True)
# rospy.sleep(3)
# res = navigate_to(x = 2, y =3, z = 1, auto_arm = False)
# rospy.sleep(5)
# res = navigate_to(x = 3, y =3.5, z = 1, auto_arm = False)