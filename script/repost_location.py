#!/usr/bin/env python
# coding=UTF-8
'''
Author: Wei Luo
Date: 2021-06-07 22:03:23
LastEditors: Wei Luo
LastEditTime: 2022-03-11 14:16:17
Note: Sometimes one may need to change the rate of the localization msgs
'''

import rospy
from geometry_msgs.msg import PoseStamped


def pose_sub_callback(msg):
    global mocap_pose
    mocap_pose = msg


if __name__ == '__main__':
    rospy.init_node('repost')
    sub_topic_name = rospy.get_param('~sub_topic_name')
    pub_topic_name = rospy.get_param('~pub_topic_name')
    sample_rate = rospy.get_param('~sample_rate')
    mocap_pose = PoseStamped()

    pose_mocap_sub = rospy.Subscriber(sub_topic_name, PoseStamped,
                                      pose_sub_callback)
    pose_pub = rospy.Publisher(pub_topic_name, PoseStamped, queue_size=10)

    rate = rospy.Rate(sample_rate)

    while not rospy.is_shutdown():
        mocap_pose.header.stamp = rospy.Time.now()
        pose_pub.publish(mocap_pose)
        rate.sleep()
