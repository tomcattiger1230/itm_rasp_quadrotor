#!/usr/bin/env python
# coding=UTF-8

from dynamixel_sdk import *  # Uses Dynamixel SDK library


class AX12A(object):

    PROTOCOL_VERSION = 1.0
    BAUDRATE = 1_000_000  # Dynamixel default baudrate
    DEVICENAME = '/dev/ttyUSB0'  # e.g 'COM3' windows or '/dev/ttyUSB0' for linux
    DEBUG = True

    def __init__(self, motor_id=1):

        self.id = motor_id

        # Control table ADDRess for AX-12
        # EEPROM REGISTER ADDRESSES - Permanently stored in memory once changed
        self.ADDR_AX_MODEL_NUMBER_L = 0
        self.ADDR_AX_MODEL_NUMBER_H = 1
        self.ADDR_AX_VERSION = 2
        self.ADDR_AX_ID = 3
        self.ADDR_AX_BAUD_RATE = 4
        self.ADDR_AX_RETURN_DELAY_TIME = 5
        self.ADDR_AX_CW_ANGLE_LIMIT_L = 6
        self.ADDR_AX_CW_ANGLE_LIMIT_H = 7
        self.ADDR_AX_CCW_ANGLE_LIMIT_L = 8
        self.ADDR_AX_CCW_ANGLE_LIMIT_H = 9
        self.ADDR_AX_SYSTEM_DATA2 = 10
        self.ADDR_AX_LIMIT_TEMPERATURE = 11
        self.ADDR_AX_MIN_LIMIT_VOLTAGE = 12
        self.ADDR_AX_MAX_LIMIT_VOLTAGE = 13
        self.ADDR_AX_MAX_TORQUE_L = 14
        self.ADDR_AX_MAX_TORQUE_H = 15
        self.ADDR_AX_RETURN_LEVEL = 16
        self.ADDR_AX_ALARM_LED = 17
        self.ADDR_AX_ALARM_SHUTDOWN = 18
        self.ADDR_AX_OPERATING_MODE = 19
        self.ADDR_AX_DOWN_CALIBRATION_L = 20
        self.ADDR_AX_DOWN_CALIBRATION_H = 21
        self.ADDR_AX_UP_CALIBRATION_L = 22
        self.ADDR_AX_UP_CALIBRATION_H = 23

        # RAM REGISTER ADDRESSES - resets after shut down
        self.ADDR_AX_TORQUE_ENABLE = 24
        self.ADDR_AX_LED = 25
        self.ADDR_AX_CW_COMPLIANCE_MARGIN = 26
        self.ADDR_AX_CCW_COMPLIANCE_MARGIN = 27
        self.ADDR_AX_CW_COMPLIANCE_SLOPE = 28
        self.ADDR_AX_CCW_COMPLIANCE_SLOPE = 29
        self.ADDR_AX_GOAL_POSITION_L = 30
        self.ADDR_AX_GOAL_POSITION_H = 31
        self.ADDR_AX_GOAL_SPEED_L = 32
        self.ADDR_AX_GOAL_SPEED_H = 33
        self.ADDR_AX_TORQUE_LIMIT_L = 34
        self.ADDR_AX_TORQUE_LIMIT_H = 35
        self.ADDR_AX_PRESENT_POSITION_L = 36
        self.ADDR_AX_PRESENT_POSITION_H = 37
        self.ADDR_AX_PRESENT_SPEED_L = 38
        self.ADDR_AX_PRESENT_SPEED_H = 39
        self.ADDR_AX_PRESENT_LOAD_L = 40
        self.ADDR_AX_PRESENT_LOAD_H = 41
        self.ADDR_AX_PRESENT_VOLTAGE = 42
        self.ADDR_AX_PRESENT_TEMPERATURE = 43
        self.ADDR_AX_REGISTERED_INSTRUCTION = 44
        self.ADDR_AX_PAUSE_TIME = 45
        self.ADDR_AX_MOVING = 46
        self.ADDR_AX_LOCK = 47
        self.ADDR_AX_PUNCH_L = 48
        self.ADDR_AX_PUNCH_H = 49

    def get_present_position(self):
        return self.get_register2(self.ADDR_AX_PRESENT_POSITION_L)

    def get_register2(self, reg_num_low):
        reg_data, dxl_comm_result, dxl_error = AX12A.packetHandler.read2ByteTxRx(
            AX12A.portHandler, self.id, reg_num_low)
        AX12A.check_error(dxl_comm_result, dxl_error)
        return reg_data

    def set_goal_position(self, goal_pos):
        """Write goal position."""
        self.set_register2(self.ADDR_AX_GOAL_POSITION_L, goal_pos)

    def set_goal_position_speed(self, goal_pos, speed):
        self.set_register4(self.ADDR_AX_GOAL_POSITION_L, goal_pos, speed)

    def set_register2(self, reg_num, reg_value):
        dxl_comm_result, dxl_error = AX12A.packetHandler.write2ByteTxRx(
            AX12A.portHandler, self.id, reg_num, reg_value)
        AX12A.check_error(dxl_comm_result, dxl_error)

    def set_register4(self, reg_num, reg_value_1, reg_value_2):
        dxl_comm_result, dxl_error = AX12A.packetHandler.write4ByteTxRx(
            AX12A.portHandler, self.id, reg_num, reg_value)
        AX12A.check_error(dxl_comm_result, dxl_error)

    def get_moving_speed(self):
        """Returns moving speed to goal position [0-1023]."""
        return self.get_register2(self.ADDR_AX_GOAL_SPEED_L)

    def set_moving_speed(self, moving_speed):
        """Set the moving speed to goal position [0-1023]."""
        self.set_register2(self.ADDR_AX_GOAL_SPEED_L, moving_speed)

        if self.DEBUG:
            self.print_status("Moving speed of ", self.id,
                              self.get_moving_speed())

    @classmethod
    def connect(cls):
        cls.open_port()
        cls.set_baudrate()
        cls.packetHandler = PacketHandler(cls.PROTOCOL_VERSION)

    @classmethod
    def open_port(cls):
        cls.portHandler = PortHandler(cls.DEVICENAME)
        if cls.portHandler.openPort():
            print("Succeeded to open the port")
        else:
            print("Failed to open the port")
            print("Press any key to terminate...")
            quit()

    @classmethod
    def set_baudrate(cls):
        if cls.portHandler.setBaudRate(cls.BAUDRATE):
            print("Succeeded to change the baudrate")
        else:
            print("Failed to change the baudrate")
            print("Press any key to terminate...")
            quit()

    @staticmethod
    def check_error(comm_result, dxl_err):
        if comm_result != COMM_SUCCESS:
            print("%s" % AX12A.packetHandler.getTxRxResult(comm_result))
        elif dxl_err != 0:
            print("%s" % AX12A.packetHandler.getRxPacketError(dxl_err))

    @staticmethod
    def print_status(dxl_property, dxl_id, value):
        print(dxl_property + "dxl ID: %d set to %d " % (dxl_id, value))
