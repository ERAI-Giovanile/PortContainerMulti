#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# @Date    : 2019-12-6 19:10:80
# @Author  : Amee
# @Version : 2.0

"""
协议如下：
+--------------------------------------------------------------------+
型号       LD242A                                                    |
包头       \xab                                                      |
地址       \x01 ~ \x0a （上电显示地址）                               |
箭头方向   \x00 (无) 0x01 (向左) 0x02 (向右)                          |
距离显示   4个字节，如123 发送 \x30\x31\x32   ASCII码                 |
对中位置   \x00 (无意义) \x01 （未对准不亮） \x02 （对准，亮绿色）     |
长条灯     \x00 (无意义) \x01 (红灯) \x02 (绿灯)                      |
包尾       \xEF                                                      |
总长度     10 byte                                                    |
+--------------------------------------------------------------------+
"""

import serial
import time


class LEDControl(object):

    def __init__(self, address=2,  # 目标LED地址
                 port='COM4',  # 串口端口号
                 baudrate=19200,  # 波特率
                 bytesize=8):  # 数据宽度

        self.address = address

        self.cmd = [0xab, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xef]

        self.cmd[1] = 0x01 * self.address

        self.cmd_dir = 0x02
        self.cmd_bar = 0x01
        self.cmd_rdy = 0x01

        self.ser = serial.Serial()
        self.ser.baudrate = baudrate
        self.ser.port = port
        self.ser.stopbits = serial.STOPBITS_ONE
        self.ser.bytesize = bytesize
        self.ser.parity = serial.PARITY_NONE
        self.ser.rtscts = 0
        self.serial_open()

        if self.serial_is_open():
            print("serial port connected!")
        else:
            print("connection failed...")

    def serial_open(self):
        """打开串口
        打开串口服务器
        """
        self.ser.open()

    def serial_is_open(self):
        """串口连接状态检查
        返回值：True - 连接正常
                False - 未连接
        """
        return self.ser.is_open

    def serial_close(self):
        """关闭串口
        手动关闭串口连接
        """
        self.ser.close()

    def serial_send(self):
        """发送数据
        把打包好的cmd通过串口发出
        """
        tem = bytearray(self.cmd)
        # print("len ",  len(tem))
        # print(tem)

        if self.serial_is_open():
            self.ser.write(tem)
        else:
            self.serial_open()

    def get_cmd(self):
        """返回打包好的cmd
        """
        return self.cmd

    def set_cmd(self, distance):

        if abs(distance) <= 5:
            self.cmd_rdy = 0x02
            self.cmd_bar = 0x02
            distance = 0
        elif abs(distance) > 999:
            distance = 999
            self.cmd_bar = 0x01
            self.cmd_rdy = 0x01
        else:
            self.cmd_bar = 0x01
            self.cmd_rdy = 0x01

        if distance > 0:
            self.cmd_dir = 0x01
        elif distance == 0:
            self.cmd_dir = 0x00
        else:
            self.cmd_dir = 0x02
        distance = abs(distance)

        self.cmd[2] = self.cmd_dir
        dd = "0000" + str(distance)
        self.cmd[3] = ord(dd[-4])
        self.cmd[4] = ord(dd[-3])
        self.cmd[5] = ord(dd[-2])
        self.cmd[6] = ord[dd[-1]]
        self.cmd[7] = self.cmd_rdy
        self.cmd[8] = self.cmd_bar

        # print(self.cmd)

        self.serial_send()

    def stand_by(self):
        self.cmd[2] = 0x00
        self.cmd[3] = 0x00
        self.cmd[4] = 0x00
        self.cmd[5] = 0x00
        self.cmd[6] = 0x00
        self.cmd[7] = 0x00
        self.cmd[8] = 0x01

    def off(self):
        self.cmd[2] = 0x00
        self.cmd[3] = 0x00
        self.cmd[4] = 0x00
        self.cmd[5] = 0x00
        self.cmd[6] = 0x00
        self.cmd[7] = 0x00
        self.cmd[8] = 0x00
        self.serial_send()


if __name__ == '__main__':

    c1 = LEDControl(port='COM1', address=1)
    # c2 = LEDControl(port="COM2", address=1)

    counter = 0

    while True:
        c1.set_cmd(4)
        # c2.set_cmd(222)

        # counter += 1
        time.sleep(1)

    c2.serial_close()
