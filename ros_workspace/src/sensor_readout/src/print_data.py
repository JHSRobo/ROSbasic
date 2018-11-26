#!/usr/bin/env python
 
import rospy
from std_msgs.msg import int64

def callback(data):
	rospy.loginfo(rospy.get_caller_id() + data.data)
     
def printer():
	sense = []
	rospy.init_node('sensor_readout', anonymous=True)
	sense.append(rospy.Subscriber("/rov/temperature", int64, callback))
	sense.append(rospy.Subscriber("/rov/pressure", int64, callback))
	sense.append(rospy.Subscriber("/rov/humidity", int64, callback))
	
	sense.append(rospy.Subscriber("/rov/yaw", int64, callback))
	sense.append(rospy.Subscriber("/rov/pitch", int64, callback))
	sense.append(rospy.Subscriber("/rov/roll", int64, callback))
	print(sense)
	rospy.spin()

if __name__ == '__main__':
	printer()
