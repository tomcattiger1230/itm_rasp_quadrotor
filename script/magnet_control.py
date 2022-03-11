#!/usr/bin/env python
# coding=UTF-8
'''
Author: Wei Luo
Date: 2022-03-11 13:34:55
LastEditors: Wei Luo
LastEditTime: 2022-03-11 14:10:36
Note: Note
'''

import rospy
from std_msgs.msg import Bool, Float32


class MagnetControl(object):
    def __init__(self,
                 magnet_topic=None,
                 servo_topic=None,
                 servo_control_mode='angle'):
        self.active_magnet = False
        if servo_control_mode in ['angle', 'torque']:
            self.servo_control_mode = servo_control_mode
        else:
            rospy.logger.error(
                'Unknown servo control mode, you should specify "angle" or "torque"'
            )

        if magnet_topic is not None:
            self.magnet_sub = rospy.Subscriber(magnet_topic, Bool,
                                               self.magnet_callback)
        if servo_topic is not None:
            self.servo_sub = rospy.Subscriber(servo_topic, Float32,
                                              self.servo_motor_callback)

    def magnet_callback(self, data):
        self.active_magnet = data.data

    def servo_motor_callback(self, data):
        self.servo_data = data.data

    def process(self):
        pass


if __name__ == '__main__':
    rospy.init_node('magnet_control')
    rate = rospy.Rate(20)  # one may need to adjust the rate later
    if rospy.has_param('~magnet_topic'):
        is_control_magnet = True
        magnet_topic = rospy.get_param('~magnet_topic')
    else:
        is_control_magnet = False
        magnet_topic = None

    if rospy.has_param('~servo_motor_topic'):
        is_control_servo = True
        servo_topic = rospy.get_param('~servo_motor_topic')
    else:
        is_control_servo = False
        servo_topic = None

    if not (is_control_magnet and is_control_servo):
        rospy.logger.info(
            "You should at least choose of the tasks of this script: 1) control onboard magnet; 2) control servo motor"
        )
    else:
        magnet_obj = MagnetControl(magnet_topic, servo_topic)

    while not rospy.is_shutdown():
        rate.sleep()
