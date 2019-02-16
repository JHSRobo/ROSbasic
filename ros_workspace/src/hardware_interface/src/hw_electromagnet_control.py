#!/usr/bin/env python
import RPi.GPIO as GPIO
import rospy
from std_msgs.msg import UInt8

def callback(data):
    if data.data == 1:
        GPIO.output(37, GPIO.HIGH)
    elif data.data == 0:
        GPIO.output(37, GPIO.LOW)

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('electromagnet')

    # PIN 37 for Electromagnet Setup
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(37, GPIO.OUT)

    rospy.Subscriber("electromagnet_control", UInt8, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
