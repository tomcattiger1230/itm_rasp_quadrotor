#!/usr/bin/env python
# coding=UTF-8
'''
Author: Wei Luo
Date: 2022-04-04 17:48:11
LastEditors: Wei Luo
LastEditTime: 2022-04-05 00:22:18
Note: Note
'''

from ax12 import Ax12
import rospy
from itm_mav_msgs.msg import itm_trajectory_msg
from std_msgs.msg import Float32
import numpy as np
from serial import Serial


class AX12Controller(Ax12):
    def __init__(self, serial_port, id=None):
        self.port = Serial(serial_port, baudrate=1000000, timeout=0.001)
        if id is None:
            id_list = self.learnServos()
            self.device_Id = id_list[0]
        else:
            self.device_Id = id
        super(AX12Controller, self).__init__()
        # subscribe trajectory
        self.sub_trajectory = rospy.Subscriber('/robot_trajectory',
                                               itm_trajectory_msg,
                                               self.traj_callback,
                                               queue_size=10)
        self.command_sub_test = rospy.Subscriber('/ax12', Float32,
                                                 self.command_test_callback)
        self.reference_alpha = np.pi / 2.0
        self.reference_alpha_rate = 0.0

    def command_test_callback(self, msg):
        self.reference_alpha = msg.data

    def traj_callback(self, msg):
        self.reference_alpha = msg.traj[0].alpha
        self.reference_alpha_rate = msg.traj[0].alpha_rate

    def progress(self, ):
        self.move(self.device_Id, self.reference_alpha)
        print(self.readPosition(self.device_Id))


if __name__ == '__main__':
    rospy.init_node('ax12_controll')
    if rospy.has_param('~serial_port'):
        serial_port = rospy.get_param('~serial_port')
    else:
        serial_port = "/dev/ttyS0"  # ttyAMA0

    ax12_obj = AX12Controller(serial_port)

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        ax12_obj.progress()
        rate.sleep()

    ax12_obj.serial_connection.close()
    rospy.loginfo('AX12 controller is closed')
