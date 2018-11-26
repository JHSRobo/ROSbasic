#!/usr/bin/env python

import rospy
import time
import math
from sense_hat import SenseHat
from std_msgs.msg import Int64
from std_msgs.msg import Float64
#from geometry_msgs.msg import Quaternion
#from tf.transformations import quaternion_from_euler 

sense = SenseHat()

def talker():
	temp_pub = rospy.Publisher('rov/temperature', Int64, queue_size = 3) #Publisher for the different sensors: Temperature, Humidity, Pressure, 
	pressure_pub = rospy.Publisher('rov/pressure', Int64, queue_size = 3) 
	humidity_pub = rospy.Publisher('rov/humidity', Int64, queue_size = 3)

	#rotation_pub = rospy.Publisher("rov/rotation", Quaternion, queue_size = 3) #rotation

	x_direction_pub = rospy.Publisher('rov/Xacceleration', Float64, queue_size = 3)#X, y and z with speed
	y_direction_pub = rospy.Publisher('rov/Yacceleration', Float64, queue_size = 3)
	z_direction_pub = rospy.Publisher('rov/Zacceleration', Float64, queue_size = 3)

	rospy.init_node('sensor_readout')
	rate = rospy.Rate(60)
	while not rospy.is_shutdown():
		temp_pub.publish(round(sense.get_temperature()))#Actually pubish the things stated above
		pressure_pub.publish(round(sense.get_pressure()))
		humidity_pub.publish(round(sense.get_humidity()))#Temperature pressure humidity

		orientation = sense.get_orientation()#roll pitch and yaw
	#	rotation_pub.publish(quaternion_from_euler(radians(orientation["yaw"]), radians(orientation["pitch"]), radians(orientation["roll"])))	#converts the degrees returned by get_orientation() to radians then uses all 4 directions into a quaternion, then publishes it

		acceleration = sense.get_accelerometer_raw()#x y and z G force, not rounded
		x_direction_pub.publish(acceleration['x'])
		y_direction_pub.publish(acceleration['y'])
		z_direction_pub.publish(acceleration['z'])
		
		rate.sleep()

if __name__ == '__main__':
	try:
		talker()
	except rospy.ROSInterruptException:
		pass
