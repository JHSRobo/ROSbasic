#!/usr/bin/env python
import RPi.GPIO as GPIO
import rospy
from std_msgs.msg import UInt8

def callback(data):
    if data.data == 2:
        GPIO.output(35, GPIO.LOW)
        GPIO.output(36, GPIO.HIGH)
    elif data.data == 3:
        GPIO.output(35, GPIO.HIGH)
        GPIO.output(36, GPIO.LOW)
    elif data.data == 4:
        GPIO.output(35, GPIO.HIGH)
        GPIO.output(36, GPIO.HIGH)
    else:
        GPIO.output(35, GPIO.LOW)
        GPIO.output(36, GPIO.LOW)

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('drive_control')

    #setup GPIO that will be used for the camera Mux
    #GPIO36(19) = A0
    #GPIO35(16) = A1
    GPIO.setmode(GPIO.BOARD)
    PIO.setup(35, GPIO.OUT)
    GPIO.setup(36, GPIO.OUT)

    rospy.Subscriber("camera_select", UInt8, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
