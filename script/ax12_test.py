#!/usr/bin/env python
# coding=UTF-8
'''
Author: Wei Luo
Date: 2022-04-06 16:14:49
LastEditors: Wei Luo
LastEditTime: 2022-05-02 16:17:06
Note: Note
'''

from ax_controller.ax12a_itm import AX12AMotorController
import time

if __name__ == '__main__':
    obj = AX12AMotorController(port_str='/dev/ttyUSB0')
    time.sleep(2)
    obj.movePosition(1, 180, degree=True)
    # time.sleep(2)
    # obj.movePosition(1, 170)
    # time.sleep(2)
    # obj.movePosition(1, 130)
    obj.readPosition(1)
    time.sleep(2)
