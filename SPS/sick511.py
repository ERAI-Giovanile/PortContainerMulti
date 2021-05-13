"""sick-511解包库
    加入多次回波解包功能
"""
import binascii
import copy
import socket
import struct
import traceback

import numpy as np
import time
from timeit import default_timer as timer
import matplotlib.pyplot as plt
import threading


class Sick511(object):

    def __init__(self, ip, port, cid=1, lidar_name="default"):
        """init

        class initial for sick511

        Arguments:
            address1 {string} -- ip address
            buffer_size {int} -- recv buffer size
        """

        self.tcp_socket = socket.socket(socket.AF_INET,
                                        socket.SOCK_STREAM)

        self.HOST = ip
        self.PORT = port
        self.cid = cid
        self.liar_name = lidar_name
        self.buffer_size = 9221
        self.x = np.zeros(1141)
        self.y = np.zeros(1141)
        self.distance = np.zeros(1141)
        self.RSSI = np.zeros(1141)
        self.realRSSI = np.zeros(1141)

        self.x_offset = 0
        self.y_offset = 0
        self.theta_offset = 0
        self.x_reverse = False
        self.y_reverse = False
        self.count = 0  # 判断扫描仪是否断线计数
        self.isDirty = False

        self._commandPackage()

        self.start_angle = -5
        self.end_angle = 185
        self.total_number = 1141

        self.channels = 1  # 回波信号通道数

        self.total_angle = (self.end_angle - self.start_angle) * np.pi / 180
        self.angle_step = self.total_angle / self.total_number
        self.theta_r = np.arange(self.start_angle,
                                 self.end_angle + self.angle_step,  # 包含终止角
                                 self.angle_step)

        self.raw = b""
        self.t = threading.Thread(target=self.getRaw)
        self.lock = threading.RLock()
        self.keep_recving = False

    def connect(self):
        """开启雷达连接
        """
        self.tcp_socket.connect((self.HOST, self.PORT))

    def isConnected(self):
        """检查雷达连接状态
        返回值为[1,1,1]
        第0位 为后两者的and  (x[1] and x[2])
        第1位 为一号雷达连接状态
        第2位 为二号雷达连接状态
        """
        self.get_state()

    def disconnect(self):
        """关闭雷达连接"""
        self.tcp_socket.close()

    def _commandPackage(self):
        """sick511控制命令包"""

        def add_frame(given):
            """命令前后添加包头包尾"""
            return b'\x02' + given + b'\x03'

        self.command = {
            # Authorised client, with password: client
            "login client":
                add_frame(b'sMN SetAccessMode 03 F4724744'),

            # Service level, with password: servicelevel
            "login service":
                add_frame(b'sMN SetAccessMode 04 81BE23AA'),

            # store all configuration to hardware
            "store config":
                add_frame(b'sMN mEEwriteall'),

            # log out and run
            "logout run":
                add_frame(b'sMN Run'),

            # Set scan configuration
            "set scan config":
                add_frame(b'sMN mLMPsetscancfg 9C4 1 683 FFFF3CB0 1C3A90'),

            # Check scan configuration
            "get scan config":
                add_frame(b'sRN LMPscancfg'),

            # Set output range
            "set output range":
                add_frame(b'sWN LMPoutputRange 1 683 FFFF3CB0 1C3A90'),

            # Check output range
            "get output range":
                add_frame(b'sRN LMPoutputRange'),

            # Set output with RSSI value
            "scan with RSSI":
                add_frame(b'sWN LMDscandatacfg 01 00 1 0 0 00 00 0 0 0 0 +1'),

            # get device state
            "get state":
                add_frame(b'sRN LCMstate'),

            # Output of measured values of one scan.
            # Sends the last valid scan data back from the memory of the LMS.
            # Also if the measurement is not running,
            # the last measurement is available.
            "pull single":
                add_frame(b'sRN LMDscandata'),

            # start to keep sending scanned value
            "pull cont":
                add_frame(b'sEN LMDscandata 1'),

            # stop to keep sending scanned value
            "pull stop":
                add_frame(b'sEN LMDscandata 0'),
        }

    def sendTcp(self, com):
        """com命令通过tcp发送"""
        self.tcp_socket.sendall(com)

    def recvTcp(self):
        """接收511，识别报尾"""
        raw = bytes()
        flag = True
        while flag:
            recved = self.tcp_socket.recv(self.buffer_size)
            #  避免……0 0 0\x03\x02sSN LMDscandata 0 1 ……场景出现
            if b"\x02" in recved or b"\x02" in raw:
                raw += recved
            if len(raw) > 1:
                if b"\x02" in raw and b"\x03" in raw:
                    flag = False
        return raw

    def getDirty(self):
        return self.isDirty

    def checkHardware(self):
        # login as client
        self.sendTcp(self.command["login client"])
        ans = self.recvTcp()
        if ans != b'\x02sAN SetAccessMode 1\x03':
            print('login error:', ans)
        else:
            print("checking login info...")

        # read scan config
        time.sleep(0.1)
        self.sendTcp(self.command["pull single"])
        ans = self.recvTcp()
        if b"\x02sRA LMDscandata" not in ans:
            print("get config from one frame error")
            return
        else:
            print("get config from one frame")
        ans = ans.split()
        isDirty = int(ans[6], 16)
        if isDirty > 1:
            self.isDirty = True
            print("lidar screen is polluted")
        else:
            self.isDirty = False
            print("lidar screen is clean")
        ds_index = ans.index(b"DIST1")
        # 扫描仪信号数据通道数，如果是1表明扫描仪处于首次回波或末次回波模式，采用所有回波模式应为5
        self.channels = int(ans[ds_index - 1], 16)
        # if ans[ds_index + 3][0] < 56:
        #     # 高位小于ascii “8”
        #     # 正数时高位0，ascii会为空，使用struct无法解包
        #     # 正数
        #     self.start_angle = int(ans[ds_index + 3], 16) / 10000
        # else:
        #     # 负数
        #     self.start_angle = struct.unpack(">i", binascii.unhexlify(ans[ds_index + 3]))[0] / 10000
        temp = binascii.unhexlify(ans[ds_index + 3])
        if len(temp) < 4:
            temp = b"\x00" * (4 - len(temp)) + temp
        self.start_angle = round(struct.unpack(">i", temp)[0] / 10000, 0)
        print("start angle: %s°" % self.start_angle)
        self.angle_step = int(ans[ds_index + 4], 16) / 10000
        print("angle step: %s°" % self.angle_step)
        self.total_number = int(ans[ds_index + 5], 16)
        print("total point numbers: %d" % self.total_number)
        self.end_angle = self.start_angle + self.angle_step * (self.total_number - 1)
        print("end angle: %s°" % self.end_angle)
        self.start_angle = self.start_angle
        self.end_angle = self.end_angle
        self.angle_step = self.angle_step
        self.theta_r = np.arange(self.start_angle * np.pi / 180,
                                 (self.end_angle + self.angle_step) * np.pi / 180,
                                 self.angle_step * np.pi / 180)
        self.x = np.zeros(self.total_number)
        self.y = np.zeros(self.total_number)
        self.distance = np.zeros(self.total_number)
        self.buffer_size = 81 + self.total_number * 8 * self.channels + 12
        time.sleep(0.1)

        # pull cont
        self.sendTcp(self.command["pull cont"])
        ans = self.recvTcp()
        if ans != b'\x02sEA LMDscandata 1\x03':
            print('data pushing error...')
            return
        else:
            print("checking pull method...")

        time.sleep(0.1)
        self.sendTcp(self.command["logout run"])
        ans = self.recvTcp()
        if b"\x02sAN Run 1\x03" != ans:
            print("set run error")
            return
        else:
            print('All setting checked! Run')

        self.startThread()
        time.sleep(2)

    def _getThetaR(self):
        return self.theta_r

    def getTheta(self):
        """
        """
        return self._getThetaR

    def getTcpSocket(self):
        return self.tcp_socket

    def getRaw(self):
        while self.keep_recving:
            raw = self.recvTcp()
            self.lockThread()
            self.raw = copy.copy(raw[raw.index(b"\x02"):raw.index(b"\x03") + 1])
            self.unlockThread()
            self.count += 1
            if self.count > 3000:
                self.count = 0

    def getCount(self):
        return self.count

    def getData(self):
        tic = timer()
        # self.lockThread()
        raw = self.raw
        # self.unlockThread()
        if b'\x02sSN LMDscandata' in raw:
            raw = raw[raw.index(b"\x02"):raw.index(b"\x03") + 1]
            splited_raw = raw.split()
            self.distance = np.zeros(self.total_number * self.channels)
            self.RSSI = np.zeros(self.total_number * self.channels)
            ds_index = [splited_raw.index(b"DIST" + str(i + 1).encode("ascii")) for i in range(self.channels)]
            r_ScaleFactor = [splited_raw[i + 1] for i in ds_index]
            # r_ScaleFactorOffset = [int(splited_raw[i + 2], 16)for i in ds_index]

            # define scale factor
            c_ScaleFactor = []
            for i in r_ScaleFactor:
                if i == b'40000000':
                    c_ScaleFactor.append(2)

                if i == b'3F800000':
                    c_ScaleFactor.append(1)
                else:
                    c_ScaleFactor.append(999)

            try:
                # ---------------deserialize distance data---------------
                for i in range(self.total_number):
                    for j in range(self.channels):
                        self.distance[i * self.channels + j] = int(splited_raw[ds_index[j] + i + 6],
                                                                   16) * c_ScaleFactor[j] / 1000
            except Exception:
                print(splited_raw.index(b"RSSI1"))
                traceback.print_exc()
                return

            # ---------------deserialize RSSI data-------------------
            rs_index = [splited_raw.index(b"RSSI" + str(i + 1).encode("ascii")) for i in range(self.channels)]
            for i in range(self.total_number):
                for j in range(self.channels):
                    self.RSSI[i * self.channels + j] = int(splited_raw[rs_index[j] + i + 6], 16)
        else:
            print('frame error!!!!')

        toc = timer()
        self.process_time = (toc - tic)
        print("inner process time: ", self.process_time)

    def get_state(self):
        self.tcp_socket.sendall(self.command.get("get_state"))
        device_state = self.tcp_socket.recv(100)
        split_device_state = device_state.split()
        state = split_device_state[2]
        state1 = state.decode()
        state2 = int(state1[0], 16)
        return state2

    def getRSSI(self):
        return self.realRSSI

    def sendHeartBeat(self):
        #  sick扫描仪不需要发送心跳，预留接口增强程序适配性
        pass

    def getXY(self):
        x_reverse_value = 1
        y_reverse_value = 1
        self.x = np.zeros(self.total_number * self.channels)
        self.y = np.zeros(self.total_number * self.channels)
        self.realRSSI = np.zeros(self.total_number * self.channels)
        if self.x_reverse:
            x_reverse_value = -1
        if self.y_reverse:
            y_reverse_value = -1
        self.getData()
        index = 0
        if not self.distance.any() or not self.RSSI.any():
            return [np.zeros(self.total_number), np.zeros(self.total_number)]
        for i in range(self.total_number):
            for j in range(self.channels):
                distance = self.distance[i * self.channels + j]
                if distance:
                    # 去除distance为0的点
                    self.x[index] = (self.distance[i * self.channels + j] *
                                     np.cos(self.theta_r[i] + self.theta_offset) + self.x_offset) * x_reverse_value

                    self.y[index] = (self.distance[i * self.channels + j] *
                                     np.sin(self.theta_r[i] + self.theta_offset) + self.y_offset) * y_reverse_value
                    self.realRSSI[index] = self.RSSI[i * self.channels + j]  # 去除这些distance为0的点的RSSI数值
                    index += 1
        #  去除数组中的空点
        self.x = np.delete(self.x, np.s_[index:], None)
        self.y = np.delete(self.y, np.s_[index:], None)
        self.realRSSI = np.delete(self.realRSSI, np.s_[index:], None)
        return [self.x, self.y]

    def getThread(self):
        """返回扫描数据线程套嵌字"""
        return self.t

    def joinThread(self):
        """关闭线程"""
        self.keep_recving = False
        self.t.join()
        print("thread joined")

    def startThread(self):
        """开启线程"""
        self.keep_recving = True
        print("starting recving thread ...")
        self.t.start()
        print("thread started ...")

    def lockThread(self):
        """需求线程锁"""
        self.lock.acquire()

    def unlockThread(self):
        """释放线程锁"""
        self.lock.release()

    def setXOffset(self, n):
        self.x_offset = n

    def getXOffset(self):
        return self.x_offset

    def setYOffset(self, n):
        self.y_offset = n

    def getYOffset(self):
        return self.y_offset

    def setThetaOffset(self, n):
        self.theta_offset = n * np.pi / 180

    def getThetaOffset(self):
        return self.theta_offset * 180 / np.pi

    def setXReverse(self, n=False):
        self.x_reverse = n

    def setYReverse(self, n=True):
        self.y_reverse = n


if __name__ == '__main__':
    IP_address = "192.168.0.10"
    port = 2111
    test = Sick511(IP_address, port)  # 16384
    test.connect()
    test.checkHardware()
    t1 = t2 = 0
    a = 0

    while 1:
        a += 1

        try:
            plt.clf()
            x, y = test.getXY()
            if x.any() and y.any():
                plt.plot(x, y, 'r.')
            print(test.channels, len(x), len(test.realRSSI), "######")
            plt.draw()
            plt.pause(0.01)
        except Exception:
            traceback.print_exc()
            test.joinThread()
            test.disconnect()
            print("---------------lidar ended------------------")
            break
