#!/usr/bin/env python
import RPi.GPIO as GPIO
import rospy
from std_msgs.msg import Bool

# Callback function that runs every time a message is published on the topic

def callback(data):
    if data.data:
        GPIO.output(38, GPIO.HIGH)
    else:
        GPIO.output(38, GPIO.LOW)

def hook():
    GPIO.cleanup()
    rospy.loginfo("Trout Grout  node shutdown successfully")

def listener():

    # Launches trout grout node
    rospy.init_node('trout_grout')

    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(38, GPIO.OUT)

    rospy.Subscriber("rov/trout_grout", Bool, callback)

    #shutdown hook releases the GPIO when the node is killed
    rospy.on_shutdown(hook)
    rospy.spin()

if __name__=='__main__':
    listener()
