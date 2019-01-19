#!/usr/bin/env python

import rospy
from drq1250.msg import DRQ1250
from PMBus import pmbus
import sys

rospy.init_node("drq2150")

myargv = rospy.myargv(sys.argv)
if len(myargv) >= 2:
    if myargv[1].startswith("0x"):
        addr = int(myargv[1][2:], 16)
    else:
        addr = int(myargv[1], 16)
else:
    rospy.logerr("USAGE WARNING: Please provide an address for the DRQ device! Using default 0x12")
    addr = 0x12

rospy.loginfo("Initializing PMBUS... ")

DRQ = pmbus(addr) #New pmbus object with device address
rospy.sleep(1)
#DRQ.setUVLimit(36.0) #Not sure if this works yet
rospy.sleep(1)

pub = rospy.Publisher("rov/drq1250", DRQ1250, queue_size=1)
rate = rospy.Rate(30)

while not rospy.is_shutdown():
    msg = DRQ1250()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = 'drq1250'
    msg.tempurature = DRQ.getTempurature()
    msg.Vin = DRQ.getVoltageIn()
    msg.Vout = DRQ.getVoltageOut()
    msg.Iout = DRQ.getCurrent()
    msg.Pout = DRQ.getPowerOut(False) #False is caclulated from given values of current and voltage while True gets values from DRQ1250

    pub.publish(msg)
    rate.sleep()
