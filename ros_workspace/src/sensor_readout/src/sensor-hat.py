#!/usr/bin/env python

import rospy
import time
import math
from sense_hat import SenseHat
from std_msgs.msg import Header
from tf.transformations import quaternion_from_euler
from sensor_msgs.msg import Imu, Temperature, RelativeHumidity, FluidPressure

sense = SenseHat()
sense.set_imu_config(True, True, True) #compass, gyro, accele
calibration = sense.get_orientation_radians()
start_roll = calibration['roll']
start_pitch = calibration['pitch']
start_yaw = calibration['yaw']
start_time = int(time.time()) * 1000

def talker():
	temp_pub = rospy.Publisher('rov/int_temperature', Temperature, queue_size = 1) #Publisher for the different sensors: Temperature, Humidity, Pressure,
	pressure_pub = rospy.Publisher('rov/int_pressure', FluidPressure, queue_size = 1)
	humidity_pub = rospy.Publisher('rov/int_humidity', RelativeHumidity, queue_size = 1)

	imu_pub = rospy.Publisher("rov/imu", Imu, queue_size = 1) #Imu publisher

	rate = rospy.Rate(60)
	while not rospy.is_shutdown():
		header = Header()
		header.stamp = rospy.Time.now()
		header.frame_id = 'sensor_data'

		temp_pub.publish(header, sense.get_temperature(), 0)#Actually pubish the things stated above
		pressure_pub.publish(header, sense.get_pressure(), 0)
		humidity_pub.publish(header, sense.get_humidity(), 0)#Temperature pressure humidity

		message = Imu() #make a new object of class IMU with name message
		message.header = header

		acceleration = sense.get_accelerometer_raw() #x y and z G force, not rounded
		message.linear_acceleration.x, message.linear_acceleration.y, message.linear_acceleration.z = (acceleration['x'] * 9.80665, acceleration['y'] * 9.80665, acceleration['z'] * 9.80665) #9.80665 is gs to newtons

		orientation = sense.get_orientation_radians() #roll pitch and yaw
		message.orientation.x, message.orientation.y, message.orientation.z, message.orientation.w = quaternion_from_euler(orientation['roll'], orientation['pitch'], orientation['yaw']) #converts the degrees returned by get_orientation() to radians then uses all 4 directions into a quaternion, then publishes it
		
		current_time = int(time.time()) * 1000
		angular_velocity_roll = (orientation['roll'] - start_roll) / ((current_time - start_time) * 1000)
		angular_velocity_pitch = (orientation['pitch'] - start_pitch) / (((current_time - start_time) * 1000)
		angular_veloctiy_yaw = (orientation['yaw'] - start_yaw) / ((current_time - start_time) * 1000)
		message.angular_velocity.x, message.angular_velocity.y, message.angular_velocity.z = (angular_velocity_roll, angular_velocity_pitch, angular_velocity_yaw)
		
		imu_pub.publish(message)

		start_roll = orientation['roll']
		start_pitch = orientation['pitch']
		start_yaw = orientation['yaw']
		start_time = int(time.time()) * 1000
		
		rate.sleep()

if __name__ == '__main__':
	try:
		rospy.init_node("sensor_interface")
		talker()
	except rospy.ROSInterruptException:
		pass
