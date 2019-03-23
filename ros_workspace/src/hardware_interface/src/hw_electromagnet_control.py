#!/usr/bin/env python
import RPi.GPIO as GPIO
import rospy
from std_msgs.msg import Bool


def callback(data):
    if data.data:
        GPIO.output(35, GPIO.HIGH)
    elif not data.data:
        GPIO.output(35, GPIO.LOW)


def hook():
    GPIO.cleanup()
    rospy.loginfo("Electromagnet node shutdown successfully")


def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('electromagnet')

    # PIN 35 for Electromagnet Setup
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(35, GPIO.OUT)

    rospy.Subscriber("electromagnet_control", Bool, callback)

    # shutdown hook releases the GPIO when the node is killed
    rospy.on_shutdown(hook)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    listener()
