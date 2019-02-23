#!/usr/bin/env python

import rospy
from ms5837.msg import ms5837_data

data = []
global time
time = 0


def calculate(array):
    mean = 0
    total = 0
    for i in range(0, len(array)):
        mean += array[i]
    mean /= len(array)
    for i in range(0, len(array)):
        total += (array[i] - mean) * (array[i] - mean)
    total /= len(array)
    print(total)


def callback(msg):
    global time
    data.append(msg.depth)
    time += 1
    print(time)
    if time >= 2000:
        calculate(data)


def main():
    rospy.init_node('test_node')
    while not rospy.is_shutdown():
        rospy.Subscriber('rov/ms5837', ms5837_data, callback)
        rospy.spin()


if __name__ == '__main__':
    main()

