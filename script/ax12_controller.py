#!/usr/bin/env python
# coding=UTF-8
'''
Author: Wei Luo
Date: 2022-04-10 18:55:02
LastEditors: Wei Luo
LastEditTime: 2022-04-10 19:26:52
Note: Note
'''

from ax_controller.ax12a_itm import AX12AMotorController
import rospy
import numpy as np
from itm_mav_msgs.msg import manipulator_state


class AX12Controller(object):
    def __init__(self, serial_port, dynamixel_id=1):
        self.serial_connection = AX12AMotorController(port=serial_port)
        # subscribe trajectory
        self.sub_trajectory = rospy.Subscriber('/robot_trajectory',
                                               itm_trajectory_msg,
                                               self.traj_callback,
                                               queue_size=10)
        # publish angle publish
        self.current_angle_degree = 0.0
        self.motor_angle_pub = rospy.Publisher(
            '/itm_manipulator_state',
            manipulator_state,
        )
        self.reference_alpha = np.pi / 2.0
        self.reference_alpha_rate = 0.0
        self.motor_id = dynamixel_id

    def traj_callback(self, msg):
        self.reference_alpha = msg.traj[0].alpha
        self.reference_alpha_rate = msg.traj[0].alpha_rate

    def progress(self, ):
        self.serial_connection.movePosition(self.motor_id,
                                            self.reference_alpha)
        self.current_angle_degree = self.serial_connection.readPosition(
            self.motor_id)
        itm_manipulator_state_obj = manipulator_state()
        itm_manipulator_state_obj.angle = self.current_angle_degree
        self.motor_angle_pub.publish(itm_manipulator_state_obj)


if __name__ == '__main__':
    ax12_obj = AX12Controller(serial_port=None)

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        ax12_obj.progress()
        rate.sleep()

    rospy.loginfo('AX12 controller is closed')
