import rospy
from uavlab411.msg import data_sensor_msg
import random
rospy.init_node("sensor_node")

pub = rospy.Publisher("/uavlab411/sensor_data",data_sensor_msg,queue_size=1)
r = rospy.Rate(3) # 10hz
data = data_sensor_msg()
data.id = 2
data.lat = 47.39816947
data.lon = 8.54610412 
data.hum = 30
data.temp = 30

data1 = data_sensor_msg()
data1.id = 1
data1.lat = 47.39785520
data1.lon = 8.54654563
data1.hum = 40
data1.temp = 40
while not rospy.is_shutdown():
   data.gas = random.randint(10,20)
   pub.publish(data)
   r.sleep()
   pub.publish(data1)
   r.sleep()
