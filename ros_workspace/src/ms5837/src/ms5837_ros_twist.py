#!/usr/bin/env python

import rospy
from ms5837.msg import ms5837_data
from geometry_msgs.msg import Twist

pub = rospy.Publisher('rov/ms5837_twist', Twist, queue_size=3)


def callback(msg):
	ros_msg = Twist()
	ros_msg.angular.z = msg.data.depth
	pub.publish(ros_msg)


def publisher():
	# set up ros stuff
	while not rospy.is_shutdown():
		rospy.init_node('ms5837_twist')
		rospy.Subscriber('rov/ms5837', ms5837_data.msg, callback)
		rospy.spin()


if __name__ == '__main__':
	try:
		publisher()
	except rospy.ROSInterruptException:
		pass
