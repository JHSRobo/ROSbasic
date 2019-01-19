#!/usr/bin/env python

import rospy
import math
from sense_hat import SenseHat
from std_msgs.msg import Header
from std_msgs.msg import Float64
from tf.transformations import quaternion_from_euler
from sensor_msgs.msg import Imu, Temperature, RelativeHumidity, FluidPressure

sense = SenseHat()
sense.set_imu_config(True, True, True) #compass, gyro, accele
calibration = sense.get_orientation_radians()
start_roll = calibration['roll']
start_pitch = calibration['pitch']
start_yaw = calibration['yaw']

#start times for integration
x_time = rospy.get_rostime()
y_time = rospy.get_rostime()
z_time = rospy.get_rostime()
roll_time = rospy.get_rostime()
pitch_time = rospy.get_rostime()
yaw_time = rospy.get_rostime()

#start vels for integration
x_vel = 0
y_vel = 0
z_vel = 0
angular_velocity_yaw = 0
angular_velocity_roll = 0
angular_velocity_pitch = 0

def talker():
	temp_pub = rospy.Publisher('rov/int_temperature', Temperature, queue_size = 1) #Publisher for the different sensors: Temperature, Humidity, Pressure,
	pressure_pub = rospy.Publisher('rov/int_pressure', FluidPressure, queue_size = 1)
	humidity_pub = rospy.Publisher('rov/int_humidity', RelativeHumidity, queue_size = 1)

	imu_pub = rospy.Publisher("rov/imu", Imu, queue_size = 1) #Imu publisher

	x_vel_pub = rospy.Publisher('rovpid/leftright/state', Float64, queue_size = 1)
	y_vel_pub = rospy.Publisher('rovpid/frontback/state', Float64, queue_size = 1)
	z_vel_pub = rospy.Publisher('rovpid/vertical/state', Float64, queue_size = 1)
	yaw_vel_pub = rospy.Publisher('rovpid/yaw/state', Float64, queue_size = 1)
	pitch_vel_pub = rospy.Publisher('rovpid/pitch/state', Float64, queue_size = 1)
	roll_vel_pub = rospy.Publisher('rovpid/roll/state', Float64, queue_size = 1)

	x_msg = Float64()
	y_msg = Float64()
	z_msg = Float64()
	yaw_msg = Float64()

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

		if (int(datetime.time.__str__(datetime.time.microsecond))) < start_time:
			start_time = start_time + 1000000

#integrate velocity
		x_vel += acceleration['x'] * ((rospy.get_rostime - prev_x_time).to_sec())
		x_time = rospy.get_rostime()
		x_msg.data = x_vel
		x_vel_pub.publish(x_msg)

		y_vel += acceleration['y'] * ((rospy.get_rostime - prev_y_time).to_sec())
		y_time = rospy.get_rostime()
		y_msg.data = y_vel
		y_vel_pub.publish(y_msg)

		z_vel += acceleration['z'] * ((rospy.get_rostime - prev_z_time).to_sec())
		z_time = rospy.get_rostime()
		z_msg.data = z_vel
		z_vel_pub.publish(z_msg)

#integrate angular velocity
		roll_vel = (orientation['roll'] - start_roll) / ((rospy.get_rostime - roll_time).to_sec())
		roll_time = rospy.get_rostime()
		start_roll = orientation['roll']
		roll_msg.data = roll_vel
		roll_vel_pub.publish(roll_msg)

		pitch_vel = (orientation['pitch'] - start_pitch) / ((rospy.get_rostime - pitch_time).to_sec())
		pitch_time = rospy.get_rostime()
		start_pitch = orientation['pitch']
		pitch_msg.data = pitch_vel
		pitch_vel_pub.publish(pitch_msg)

		yaw_vel = (orientation['yaw'] - start_yaw) / ((rospy.get_rostime - yaw_time).to_sec())
		yaw_time = rospy.get_rostime()
		start_yaw = orientation['yaw']
		yaw_msg.data = yaw_vel
		yaw_vel_pub.publish(yaw_msg)

		message.angular_velocity.x, message.angular_velocity.y, message.angular_velocity.z = (angular_velocity_roll, angular_velocity_pitch, angular_velocity_yaw)

		imu_pub.publish(message)

		rate.sleep()

if __name__ == '__main__':
	try:
		rospy.init_node("sensor_interface")
		talker()
	except rospy.ROSInterruptException:
		pass
