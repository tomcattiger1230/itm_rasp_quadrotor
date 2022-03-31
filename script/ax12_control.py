#!/usr/bin/env python
# coding=UTF-8
'''
Author: Wei Luo
Date: 2022-04-01 00:38:00
LastEditors: Wei Luo
LastEditTime: 2022-04-01 00:42:11
Note: Note
'''

import rospy
import numpy as np
from pyax12.connection import Connection


class AX12Controller(object):
    def __init__(self, serial_port, dynamixel_id=1):
        # Connect to the serial port
        self.serial_connection = Connection(port=serial_port, baudrate=57600)
        # incase multiple dynamixel motors
        self.dynamixel_id = dynamixel_id


if __name__ == '__main__':
    rospy.init_node('ax12_controll')
    if rospy.has_param('~serial_port'):
        serial_port = rospy.get_param('~serial_port')
    else:
        serial_port = "/dev/ttyS0"  # ttyAMA0

    ax12_obj = AX12Controller(serial_port)

    while not rospy.is_shutdown():
        pass

    rospy.loginfo('AX12 controller is closed')
