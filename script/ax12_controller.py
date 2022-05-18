#!/usr/bin/env python
# coding=UTF-8
'''
Author: Wei Luo
Date: 2022-04-10 18:55:02
LastEditors: Wei Luo
LastEditTime: 2022-05-18 15:18:42
Note: Note
'''

from ax_controller.ax12_config import AX12A
import rospy
import numpy as np
from itm_mav_msgs.msg import manipulator_state, itm_trajectory_msg


class AX12Controller(object):
    def __init__(self, dynamixel_id=1, pos_rate_control=False):
        # e.g 'COM3' windows or '/dev/ttyUSB0' for Linux
        AX12A.DEVICENAME = '/dev/ttyUSB0'

        AX12A.BAUDRATE = 1_000_000

        # sets baudrate and opens com port
        AX12A.connect()

        self.serial_connection = AX12A(dynamixel_id)

        # subscribe trajectory
        self.sub_trajectory = rospy.Subscriber('/robot_trajectory',
                                               itm_trajectory_msg,
                                               self.traj_callback,
                                               queue_size=10)
        # publish angle, topicname '/itm_manipulator_state', topictype manipulator_state
        self.current_angle_degree = 0.0
        self.motor_angle_pub = rospy.Publisher('/itm_manipulator_state',
                                               manipulator_state,
                                               queue_size=10)
        self.reference_alpha = 0.0
        self.reference_alpha_pos = self.rad_to_pos(self.reference_alpha)
        # self.reference_alpha_rate = 1
        # self.reference_alpha_rate_pos = self.rpm_to_ratepos(self.reference_alpha_rate)
        self.motor_id = dynamixel_id
        self.itm_manipulator_state_obj = manipulator_state()
        self.pos_rate_control = pos_rate_control

    def traj_callback(self, msg):
        self.reference_alpha = msg.traj[1].alpha
        if self.pos_rate_control:
            self.reference_alpha_rate = msg.traj[0].alpha_rate
        # print(self.reference_alpha)
        # self.reference_alpha_pos = self.rad_to_pos(self.reference_alpha)

        # self.reference_alpha_rate_pos = self.rpm_to_ratepos(self.reference_alpha_rate)

    def progress(self, ):
        if self.pos_rate_control:
            self.serial_connection.set_goal_position_speed(
                self.rad_to_pos(self.reference_alpha),
                self.rad_per_second_to_ratepos(
                    np.abs(self.reference_alpha_rate)))
            rospy.loginfo_once("Using angle and angular rate command")
        else:
            self.serial_connection.set_goal_position(
                self.rad_to_pos(self.reference_alpha))
            rospy.loginfo_once("Using angle command")

        # get the current angle of the manip.
        self.current_angle_degree = self.pos_to_rad(
            self.serial_connection.get_present_position())

        # publish the current angle of the manip.
        self.itm_manipulator_state_obj.angle = self.current_angle_degree
        self.itm_manipulator_state_obj.header.stamp = rospy.Time.now()
        self.motor_angle_pub.publish(self.itm_manipulator_state_obj)

    @staticmethod
    def rpm_to_ratepos(rpm):
        ratepos = int(rpm / 114 * 1023)
        return ratepos

    @staticmethod
    def rad_per_second_to_ratepos(rad_speed):
        ratepos = int(rad_speed * 30 / np.pi / 114 * 1023)
        return ratepos

    @staticmethod
    def degree_to_pos(degree):
        pos = int((degree + 60) / 300 * 1023)
        return pos

    @staticmethod
    def pos_to_degree(pos):
        degree = (pos * 300 / 1023) - 60
        return degree

    @staticmethod
    def rad_to_pos(rad):
        pos = int((rad + 1 / 3 * np.pi) / (300 / 180 * np.pi) * 1023)
        return pos

    @staticmethod
    def pos_to_rad(pos):
        rad = (pos * (300 / 180 * np.pi) / 1023) - 1 / 3 * np.pi
        return rad


if __name__ == '__main__':
    rospy.init_node('ax12a_controller')
    if rospy.has_param('~has_alpha_rate'):
        has_alpha_rate = rospy.get_param('~has_alpha_rate')
    else:
        has_alpha_rate = False

    ax12_obj = AX12Controller(1, pos_rate_control=has_alpha_rate)

    rate = rospy.Rate(100)

    while not rospy.is_shutdown():
        ax12_obj.progress()
        rate.sleep()

    rospy.loginfo('AX12 controller is closed')
