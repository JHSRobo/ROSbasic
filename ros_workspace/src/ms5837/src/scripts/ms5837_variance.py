#!/usr/bin/env python

import rospy
from ms5837.msg import ms5837_data

data = []
global time
time = 0


def calculate(array):
    mean = 0
    total = 0
    for point in array:
        mean += point
    mean /= len(array)
    for point in array:
        total += (point - mean) * (point - mean)
    total /= len(array)
    print(total)


def callback(msg):
    data.append(msg.data.depth)
    time += 1
    if time >= 2000:
        calculate(data)


def main():
    while not rospy.is_shutdown():
        rospy.Subscriber('rov/ms5837', ms5837_data.msg, callback)
        rospy.spin()


if __name__ == '__main__':
    main()
