#!/usr/bin/python3
# -*- coding: utf-8 -*-
# @Date    : 2019-01-4 08:53:19
# @Author  : Wy
# @Version : 1.0

"""
    LIM_TAG = 0xF5EC96A5  # 报文开头
    LIM_VER = 0x01000000  # 报文版本
    LIM_DATA_LEN = 4  # 数据长度

    LIM_CODE_LDBCONFIG = 111        # 设备配置信息.设备端发送
    LIM_CODE_START_LDBCONFIG = 110        # 启动设备配置信息广播.应用端发送
    LIM_CODE_STOP_LDBCONFIG = 112        # 停止设备配置信息广播.应用端发送
    LIM_CODE_GET_LDBCONFIG = 114        # 获取设备配置信息.应用端发送

    LIM_CODE_HB = 10              # 心跳.应用端发送
    LIM_CODE_HBACK = 11              # 心跳回复.设备端发送
    LIM_CODE_LMD = 901             # LMD：雷达测量数据，参考下文LMD的说明.设备端发送
    LIM_CODE_LMD_RSSI = 911             # LMD-RSSI：带反射率的雷达测量数据.设备端发送
    LIM_CODE_START_LMD = 1900            # 请求LMD数据代码.应用端发送
    LIM_CODE_STOP_LMD = 1902            # 停止LMD数据代码.应用端发送
    LIM_CODE_ALARM = 9001            # 运行状态报警，参考下文ALARM/DISALARM的说明.设备端发送
    LIM_CODE_DISALARM = 9003            # 状态报警取消，参考下文ALARM/DISALARM的说明.设备端发送

    LIM_CODE_FMSIG_QUERY = 1912            # 查询区域监测信号.应用端发送
    LIM_CODE_FMSIG = 1911            # 区域监测信号.设备端发送

#  去掉 显示扫描仪位置的点
"""



# from import_collector import *

import socket
import numpy as np
import struct
import time
import traceback
import threading
import os
# from customError import CustomError
#from color import Color

# from toolbox import toolbox
#from import_collector import *
##
# just a brif
#


class FM_TCP(object):
    """
    FM_TCP()

    飞思迈尔TCP解包库

    """

    def __init__(self, ip=1, port=1, cid=1, lidar_name="default"):
        """
         init :ip, port, cid , lidar_name
         defaul value ：1,1,1, 'default'

         default : tcp socket
         Args:
            ip: something ip.
            port: something port
            cid: something cid
            auto_connect: something about the connection
        """
        #self.cp = Color()
        self.TCP_IP = ip
        self.TCP_PORT = port

        self.initSendStructure()
        self.initRecvStructure()
        self.LIM_CID = cid
        self.lidar_name = lidar_name

        self.tcp_socket = socket.socket(socket.AF_INET,
                                        socket.SOCK_STREAM)
        self.raw_data = []
        self.count = 0 # 计数判断扫描仪是否卡死
        self.offset_x = 0
        self.offset_y = 0
        self.offset_theta = 0

        self.x_reverse = False
        self.y_reverse = False
        self.isDirty = False

        self.t = threading.Thread(target=self.getRaw)
        self.lock = threading.RLock()
        self.keep_recving = False
        self.RSSI = []

        self.CODE_LMD = b'\x8f\x03\x00\x00'

    def initSendStructure(self):
        """init the sending data structure
        """
        self.LIM_TAG = 0xF5EC96A5
        self.LIM_VER = 0x01000000
        self.LIM_CID = 1
        self.LIM_CODE = 1
        self.LIM_DATA_1 = 0
        self.LIM_DATA_2 = 0
        self.LIM_DATA_3 = 0
        self.LIM_DATA_4 = 0
        self.LIM_LEN = 40
        self.LIMsetChecksum = 0

    def initRecvStructure(self):
        """init the recivign data structure
        """
        self.LIM_SEND = b''
        self.LIM_REC_HEAD = b''
        self.LIM_REC_DATA_HEAD = b''
        self.LIM_REC_DATA = b''

        self.REC_HEAD_TAG = b''
        self.REC_HEAD_VER = b''
        self.REC_HEAD_CID = b''
        self.REC_HEAD_CODE = b''
        self.REC_HEAD_DATA1 = b''
        self.REC_HEAD_DATA2 = b''
        self.REC_HEAD_DATA3 = b''
        self.REC_HEAD_DATA4 = b''
        self.REC_HEAD_LEN = b''
        self.REC_HEADsetChecksum = b''
        self.recved = b''
        self.recved_thread = b''
        self.recved_last_correct = b''
        self.REC_DATA_HEAD_DATA_NUM = b''

        self.REC_DATA_DIS = []

    def connect(self):
        """start connection via tcp socket
        """
        self.tcp_socket.connect((self.TCP_IP, self.TCP_PORT))

    def disconnect(self):
        """disconnect tcp socket
        """
        self.tcp_socket.close()

    def int2Bytes(self, input_int):
        """整形转为4个长度，little endian的byte
        """
        return input_int.to_bytes(4, byteorder='little')

    def setLimCode(self, arg):
        """生成预设的发送命令代码
           这里后面可以改成字典键值对存储的形式，速度会比if判断更快
        """
        if arg == "hb":
            self.LIM_CODE = 10
        elif arg == "start":
            self.LIM_CODE = 1900
        elif arg == "stop":
            self.LIM_CODE = 1902
        elif arg == "GET_LDBCONFIG":
            self.LIM_CODE = 114
        elif arg == 'LDBCONFIG':
            self.LIM_SEND = 111

    def setChecksum(self):
        """计算发送的checksum
        """
        tem = self.LIM_TAG
        tem ^= self.LIM_VER
        tem ^= self.LIM_CODE
        tem ^= self.LIM_DATA_1
        tem ^= self.LIM_DATA_2
        tem ^= self.LIM_DATA_3
        tem ^= self.LIM_DATA_4
        tem ^= self.LIM_LEN
        self.LIMsetChecksum = tem

    def serialize(self, arg):
        """发送命令打包
        """
        self.initSendStructure()
        self.setLimCode(arg)
        self.setChecksum()
        tem = self.int2Bytes(self.LIM_TAG)
        tem += self.int2Bytes(self.LIM_VER)
        tem += self.int2Bytes(self.LIM_CID)
        tem += self.int2Bytes(self.LIM_CODE)
        tem += self.int2Bytes(self.LIM_DATA_1)
        tem += self.int2Bytes(self.LIM_DATA_2)
        tem += self.int2Bytes(self.LIM_DATA_3)
        tem += self.int2Bytes(self.LIM_DATA_4)
        tem += self.int2Bytes(self.LIM_LEN)
        tem += self.int2Bytes(self.LIMsetChecksum)
        self.LIM_SEND = tem
        # print(self.LIM_SEND)

    def sendHeartBeat(self):
        """send hear beat message
        """
        # self.initSendStructure()
        self.serialize('hb')
        self.sendPackage()

    def getConfig(self):
        ''' get begin / end angle ,data length from current configuration :
        '''
        print("reading lidar hardware setting...")
        self.serialize('GET_LDBCONFIG')
        self.sendPackage()
        # time.sleep(2)
        _ = self.tcp_socket.recv(10000)
        self.serialize('GET_LDBCONFIG')
        self.sendPackage()
        # time.sleep(2)
        loop_count = 10

        while loop_count:
            print("seeking config in main loop:", loop_count)
            cfg = self.tcp_socket.recv(10000)
            # print(cfg)
            print(len(cfg))
            if b'\x29\x23\x00\x00\x65' in cfg:
                self.isDirty = True
            if b'\x2b\x23\x00\x00' in cfg:
                self.isDirty = False

            if b'\x6f\x00\x00\x00' not in cfg:
                loop_count -= 1
                continue
            else:
                break

        if not loop_count and b'\x6f\x00\x00\x00' not in cfg:
            print('failed to get configuration......')
            # raise Exception('failed to get configuration')
            os._exit(0)
        else:
            pos = cfg.find(b'\x6f\x00\x00\x00')

            self.BAngle = self.__hex2int(cfg[pos+336:pos+340])
            self.EAngle = self.__hex2int(cfg[pos+340:pos+344])

        self.DataLen = int((self.EAngle - self.BAngle)/0.25 + 1)
        print('begin angle: ', self.BAngle)
        print('eng angle:   ', self.EAngle)
        print('data length: ', self.DataLen)

        self.start_theta = self.BAngle * np.pi / 180
        self.theta_step = 0.25 * np.pi / 180
        self.theta = [ele * self.theta_step +
                      self.start_theta for ele in range(self.DataLen)]

    def __hex2int(self, hex_byte):
        '''convert hex bytes to signed decimal
            example :"\xe8\x6e\x03\x00" -> 225,000
        '''
        # little-endian and signed long type
        result = struct.unpack('<l', hex_byte)[0]
        return result

    def getStartAngle(self):
        '''get the start angle'''
        return self.BAngle

    def getEndAngle(self):
        '''get the end angle'''
        return self.EAngle

    def getDtaLen(self):
        '''get the length of data'''
        return self.DataLen

    def sendStart(self):
        """begin scanning
        """
        self.getConfig()
        time.sleep(1)
        self.startThread()
        self.serialize("start")
        self.sendPackage()
        time.sleep(1)

    def sendStop(self):
        """stop scanning
        """
        self.serialize("stop")
        self.sendPackage()

    def sendPackage(self):
        """send packages via tcp
        """
        self.tcp_socket.sendall(self.LIM_SEND)

    def getRaw(self):
        '''get the raw data from tcp socket
        '''
        print(self.lidar_name, "strat!!!!!!!!!!!!!!!!!!!!!!!!!!")
        try:
            while self.keep_recving:
                required_len = 20
                self.count += 1
                while required_len < 40:
                    try:
                        head = self.tcp_socket.recv(40)
                        required_len = struct.unpack("<2H", head[32:36])[0]
                    except:
                        pass

                if required_len == 40:
                    body = b''
                else:
                    body = self.tcp_socket.recv(required_len - 40)
                    body_len = len(body)
                    if body_len < required_len - 40:
                        body += self.tcp_socket.recv(required_len -
                                                     40 - body_len)

                self.lockThread()
                self.recved_thread = head+body
                self.unlockThread()

                if self.count > 3000:
                    self.count = 0
        except:
            print(self.lidar_name, '连接线程已结束！！！！！！！！！！！！！！！！！！！！！！！！')
            # self.disconnect()
            # raise CustomError(self.self.lidar_name)

    def getCount(self):
        return self.count

    def recvPackage(self):
        """receive the package via tcp socket
        """
        flag = 0
        REC_HEAD_CODE = b''
        self.recved = self.recved_thread[:]
        recved_len = len(self.recved)
        # print("tcp recved: %d" % recved_len)

        if len(self.recved) >= 40:
            REC_HEAD_LEN = struct.unpack("<2H", self.recved[32:36])[0]
            # print("length from head: %d" % REC_HEAD_LEN)
            REC_HEAD_CODE = self.recved[12:16]
            # print('rec_head_code', REC_HEAD_CODE)

        if REC_HEAD_CODE == self.CODE_LMD:

            self.recved_last_correct = self.recved[:]
            self.deserializeRSSI(
                self.recved[64 + self.DataLen * 2:64+self.DataLen * 4])
            self.deserializeData(self.recved[64:64 + self.DataLen * 2])
        else:
            self.deserializeRSSI(
                self.recved[64 + self.DataLen * 2:64+self.DataLen * 4])
            self.deserializeData(
                self.recved_last_correct[64:64 + self.DataLen * 2])
        # 镜面污染报警信息
        if b'\x29\x23\x00\x00\x65' in self.recved:
            self.isDirty = True
            print('！！！！！！！！！！！！！！！！污染！！！！！！！！！！！！！！！！！！')
        elif b'\x2b\x23\x00\x00' in self.recved:
            self.isDirty = False

    def deserialize(self):
        """undo the packages to get head data
        """

        self.LIM_REC_HEAD = self.recved[:40]
        self.deserializeHead()
        self.LIM_REC_DATA_HEAD = self.recved[40:64]
        self.deserializeDataHead()
        self.LIM_REC_DATA = self.recved[64:self.DataLen * 2+64]
        self.RSSI = self.recved[self.DataLen*2+64:self.DataLen*4+64]

        self.deserializeData(self.LIM_REC_DATA)
        # print(self.LIM_REC_DATA)
        self.deserializeRSSI(self.RSSI)

    def deserializeHead(self):
        """undo the data head
        """
        # print(self.recved[32:36])
        self.REC_HEAD_TAG = self.recved[:4]
        self.REC_HEAD_VER = self.recved[4:8]
        self.REC_HEAD_CID = self.recved[8:12]
        self.REC_HEAD_CODE = self.recved[12:16]
        self.REC_HEAD_DATA1 = self.recved[16:20]
        self.REC_HEAD_DATA2 = self.recved[20:24]
        self.REC_HEAD_DATA3 = self.recved[24:28]
        self.REC_HEAD_DATA4 = self.recved[28:32]
        self.REC_HEAD_LEN = struct.unpack("<2H", self.recved[32:36])[0]
        self.REC_HEADsetChecksum = self.recved[36:40]

        # print(tem)

    def deserializeDataHead(self):
        """Todo
        """
        self.REC_DATA_HEAD_DATA_NUM = struct.unpack(
            "<2H", self.recved[60:64])[0]
        #tem = struct.unpack("<2H", self.recved[40:64])[0]
        print("len from data head: %d" % self.REC_DATA_HEAD_DATA_NUM)

    def deserializeData(self, input):
        """filter the boundary data
        """
        # print('高亮点的长度：：：',len(self.REC_DATA_RSSI))
        self.REC_DATA_DIS = []
        for i in range(len(input) // 2):
            temp = struct.unpack("<H", input[i*2: i*2+2])[0]
            if temp == 10001:
                self.REC_DATA_DIS.append(0)
            else:
                self.REC_DATA_DIS.append(temp)
        # print("数据长度：：", len(self.REC_DATA_DIS))


    def deserializeRSSI(self, input):
        self.REC_DATA_RSSI = []
        for i in range(len(input) // 2):
            temp = struct.unpack("<H", input[i*2: i*2+2])[0]
            self.REC_DATA_RSSI.append(temp)
        # print('RSSI:', self.REC_DATA_RSSI)
        # print('the max :', max(self.REC_DATA_RSSI))
        # print('RSSI len:', len(self.REC_DATA_RSSI))

    def getXY(self):
        """
        polar to XY cord..

        Args:
            offset_theta: theta offset
            offset_x: x offset
            offset_y: y offset
        """
        self.recvPackage()
        x = np.empty(len(self.REC_DATA_DIS))
        y = np.empty(len(self.REC_DATA_DIS))

        x_reverse_value = 1
        y_reverse_value = 1

        if self.x_reverse:
            x_reverse_value = -1
        if self.y_reverse:
            y_reverse_value = -1
        for i in range(len(self.REC_DATA_DIS)):

            # x[i] = (self.REC_DATA_DIS[i]/100 * np.cos(self.theta[i]) +
            #         self.offset_x) * x_reverse_value
            # y[i] = (self.REC_DATA_DIS[i]/100 * np.sin(self.theta[i]) +
            #         self.offset_y) * y_reverse_value
            x[i] = (self.REC_DATA_DIS[i]/100 * np.cos(self.theta[i] +
                                                      self.offset_theta) +
                    self.offset_x) * x_reverse_value
            y[i] = (self.REC_DATA_DIS[i]/100 * np.sin(self.theta[i] +
                                                      self.offset_theta) +
                    self.offset_y) * y_reverse_value
        # print(x)
        return x, y

    def getRSSI(self):
        return self.REC_DATA_RSSI

    def getDirty(self):
        return self.isDirty

    def getThread(self):
        return self.t

    def joinThread(self):
        self.keep_recving = False
        self.t.join()
        print("%s thread joined" % self.lidar_name)

    def startThread(self):
        """start Thread"""
        self.keep_recving = True
        print("starting %s recving thread ..." % self.lidar_name)
        self.t.start()
        print("%s thread started." % self.lidar_name)

    def lockThread(self):
        """lock Thread"""
        self.lock.acquire()

    def unlockThread(self):
        """release Lock """
        self.lock.release()

    def setXOffset(self, n):
        self.offset_x = n

    def getOffsetX(self):
        return self.offset_x

    def setYOffset(self, n):
        self.offset_y = n

    def getOffsetY(self):
        return self.offset_y

    def setThetaOffset(self, n):
        self.offset_theta = n * np.pi / 180

    def getOffsetTheta(self):
        return self.offset_theta * 180 / np.pi

    def setXReverse(self, n=False):
        self.x_reverse = n

    def setYReverse(self, n=True):
        self.y_reverse = n


if __name__ == '__main__':

    import matplotlib.pyplot as plt

    test = FM_TCP('10.14.8.17', 2112, 10, "lidar 1")
    rssi = []

    # test.setThetaOffset(0)
    test.setYReverse()
    test.setYOffset(0)
    test.setThetaOffset(0)

    test.connect()
    # test.getRaw()
    # print('**********',test.recved_thread)
    # get the end & start angle & data length from configuration package
    # test.getConfig()

    test.sendStart()
    test.setYReverse(True)
    a = 1
    # time.sleep(1)

    while 1:
        try:

            # print("--------------------%d------------------" % a)
            a += 1
            if a > 10:
                test.sendHeartBeat()
                # test.sendStart(False)
                a = 0
            # test.recvPackage()
            # time.sleep(2)

            # test.recvPackage()
            # test.deserialize()

            x, y = test.getXY()
            for index in range(len(x)):
                if -0.5 < x[index] < 0.5 and -0.5 < y[index] < 0.5:
                    print("The Point : ", x[index],y[index])

            # rssi = test.getRSSI()
            # rssi_x = []
            # rssi_y = []
            # for i in range(len(rssi)):
            #     if rssi[i] > 50:
            #         rssi_x.append(x[i])
            #         rssi_y.append(y[i])

            # print('RSSI:', len(rssi_x))
            # if len(x) < 700:
            #     print('x:', len(x))
            #     time.sleep(10)

            #y = toolbox.medFilter(y, 7)
            #print("length x, length y: %d %d" % (len(x), len(y)))
            # if (len(x) != 1081):
            #     print("######length error#######")
            #     print("with length x, length y: %d %d" % (len(x), len(y)))
            #     print(test.LIM_REC_HEAD)
            #     print("========================")
            #     print(test.recved)
            #     time.sleep(1)
            #     break

            plt.clf()
            print("recved distance data len: %d" % len(x))
            plt.axis([-70, 50, -5, 30])
            tic = time.time()
            plt.plot(x, y, 'r.')
            # plt.plot(rssi_x, rssi_y, 'b*')
            plt.draw()
            plt.pause(0.001)
            # time.sleep(2)
            toc = time.time()
            # print(toc-tic)
        except:
            # print(e)
            traceback.print_exc()
            test.joinThread()
            test.disconnect()
            print("----------%s lidar ended----------" % test.lidar_name)
            break


'''
优化了deserializeData的循环
增加了getConfig方法，获取配置信息
通过getStartAngle(self) getEndAngle(self) getDtaLen(self) 获取开始角度和结束角度信息，data长度信息。
优化了recvdPackage方法,
theta角度信息由config信息决定

'''
