#!/usr/bin/env python
import RPi.GPIO as GPIO
import rospy
from std_msgs.msg import Bool

# Callback function that runs every time a new message is published on the topic
# If true, sets pin 37 to high, if false sets pin 37 to low
def callback(data):
    if data.data:
        GPIO.output(37, GPIO.HIGH)
    else:
        GPIO.output(37, GPIO.LOW)

def listener():

    # Launches trout grout node
    rospy.init_node('trout_grout')

    #setup GPIO that will be used for the motor

    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(37, GPIO.OUT)

    rospy.Subscriber("rov/trout_grout", Bool, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()


