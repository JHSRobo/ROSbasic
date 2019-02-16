#!/usr/bin/env python

import rospy
from ms5837.msg import ms5837_data
from geometry_msgs.msg import Twist


pub = rospy.Publisher('rov/ms5837_twist', Twist, queue_size=3)
global zero_value
zero_value = 0
ros_msg = Twist()
ros_msg.angular.z = 0


def callback(msg):
	ros_msg.angular.z = msg.data.depth - zero_value
	pub.publish(ros_msg)


def zeroing():
	zero_value += ros_msg.angular.z


def publisher():
	# set up ros stuff
	while not rospy.is_shutdown():
		rospy.init_node('ms5837_twist')
		rospy.Service('zero_depth_sensor', None, zeroing)
		rospy.Subscriber('rov/ms5837', ms5837_data.msg, callback)
		rospy.spin()


if __name__ == '__main__':
	try:
		publisher()
	except rospy.ROSInterruptException:
		pass
