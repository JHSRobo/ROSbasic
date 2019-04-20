#!/usr/bin/python

import io  # used to create file streams
import rospy
from std_msgs.msg import Float64


class AtlasI2C:
    long_timeout = 1.5  # the timeout needed to query readings and calibrations
    short_timeout = .5  # timeout for regular commands
    default_bus = 1  # the default bus for I2C on the newer Raspberry Pis, certain older boards use bus 0
    default_address = 98  # the default address for the sensor
    current_addr = default_address

    def __init__(self, address=default_address, bus=default_bus):
        # open two file streams, one for reading and one for writing
        # the specific I2C channel is selected with bus
        # it is usually 1, except for older revisions where its 0
        # wb and rb indicate binary read and write
        self.file_read = io.open("/dev/i2c-" + str(bus), "rb", buffering=0)
        self.file_write = io.open("/dev/i2c-" + str(bus), "wb", buffering=0)

    def read(self, num_of_bytes=31):
        # reads a specified number of bytes from I2C, then parses and displays the result
        res = self.file_read.read(num_of_bytes)  # read from the board
        if type(res[0]) is str:  # if python2 read
            response = [i for i in res if i != '\x00']
            if ord(response[0]) == 1:  # if the response isn't an error
                # change MSB to 0 for all received characters except the first and get a list of characters
                # NOTE: having to change the MSB to 0 is a glitch in the raspberry pi, and you shouldn't have to do this!
                char_list = list(map(lambda x: chr(ord(x) & ~0x80), list(response[1:])))
                return "Command succeeded " + ''.join(char_list)  # convert the char list to a string and returns it
            else:
                return "Error " + str(ord(response[0]))

        else:  # if python3 read
            if res[0] == 1:
                # change MSB to 0 for all received characters except the first and get a list of characters
                # NOTE: having to change the MSB to 0 is a glitch in the raspberry pi, and you shouldn't have to do this!
                char_list = list(map(lambda x: chr(x & ~0x80), list(res[1:])))
                return "Command succeeded " + ''.join(char_list)  # convert the char list to a string and returns it
            else:
                return "Error " + str(res[0])


def main():
    device = AtlasI2C()  # creates the I2C port object, specify the address or bus if necessary
    # main loop
    pub = rospy.Publisher('sen10972', Float64, queue_size=5)
    rospy.init_node('sen10972', anonymous=True)
    rate = rospy.Rate(10)  # 10hz
    while not rospy.is_shutdown():
        msg = Float64()
        msg.data = device.read()
        pub.publish(msg)
        rate.sleep()


if __name__ == '__main__':
    main()
