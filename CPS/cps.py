#!/usr/bin/env python3
# CODING:UTF-8
# _*_ @Author:Amee _*_
# _*_ @Date:2019-10-29 _*_
# _*_ Project : chongqing guoyuan _*_

"""
todo
解决库的数据为空或者不足问题
解决放箱位置选择问题
解决161- 171 131 的offset 偏移问题


前箱有箱，放后箱位置不够，
修改有箱情况下的引导值计算方式，增加一个引导值，求平均值
加入箱门朝向

绝对值 使用存在问题？
发给PLC的距离最后要取整
日志记录加入所有涉及到的距离值

error_count, lidar_reconnect_count, plc_reconnect_count 超过10次退出重启
__init__方法加入try 捕获写入log
"""

import matplotlib.pyplot as plt
from FreeMeasureC401 import FM_TCP
import threading
import json
from socket import socket, AF_INET, SOCK_STREAM
import sys
import traceback
import numpy as np
from time import sleep, time, strftime, localtime
import struct
from datetime import datetime
from FeedDog import FeedWatchDog

from logger_util import UniversalLogger
from LEDControllor import LEDControl

# 初始化看门狗程序，本地端口为9998
FD = FeedWatchDog(9998)
# 第一次喂狗
FD.feed()

STDOUT_FMT = \
    "+------------------------{0}------------------------------+\n" \
    "工况:{1}\n" \
    "司机室前侧：【车尾】 {2} 【箱尾】 {3} 【箱头】 {4} 【距离】 {5}\n" \
    "司机室后侧：【车尾】 {6} 【箱尾】 {7} 【箱头】 {8} 【距离】 {9}\n" \
    "循环用时{10}\n" \
    "+-------------------------------------------------------------+"


# 对两个列表进行排序
def sortFirstElement(x, y):
    newtuple = [(x, y) for x, y in zip(x, y)]

    output_x = []
    output_y = []
    for ele in sorted(newtuple):
        output_x.append(ele[0])
        output_y.append(ele[1])

    return output_x, output_y


class CPSSystem(object):

    def __init__(self):
        try:
            self.log = UniversalLogger("cps").logger
            self.lidarB = None
            self.lidarF = None
            # 前侧led显示器
            self.ledF = None
            # 后侧led显示器
            self.ledB = None
            self.config = self.loadCfgFile()

            self.lidarB_reconnect_count = 0  # 司机室后侧扫描仪重连计数
            self.lidarF_reconnect_count = 0  # 司机室前侧扫描仪重连计数
            self.plc_reconnect_count = 0  # PLC断线重连计数
            self.led_reconnect_count = 0  # 显示器重连次数计数

            self.ft20Len = self.config["ft20Len"]
            self.ft40Len = self.config["ft40Len"]
            self.ft45Len = self.config["ft45Len"]
            self.gapLen = self.config["gapLen"]
            self.lidarBState = True  # 监测司机室后侧扫描仪的状态，离线为False
            self.lidarFState = True  # 监测司机室前侧扫描仪的状态
            self.plcState = True
            self.PLC_socket = None
            self.lidarB_last_count = 0
            self.lidarF_last_count = 0
            if self.config["guideChoice"] > 0:
                self.initLidarF()
                self.lidarF_last_count = self.lidarF.getCount()
            if self.config["guideChoice"] < 0 or self.config["guideChoice"] > 1:
                self.initLidarB()
                self.lidarB_last_count = self.lidarB.getCount()
            self.refreshAllLidar()

            self.plt = plt
            if self.config["PLCSwitch"]:
                self.initPLC()
            if self.config.get("ledFAddr"):
                self.ledF = LEDControl(1, self.config["ledFAddr"])
            if self.config.get("ledBAddr"):
                self.ledB = LEDControl(1, self.config["ledBAddr"])
            #  communication with IRC via threading
            self.t = threading.Thread(target=self._threadHandler)
            self.lock = threading.Lock()
            self.thread_switch = False
            self.recMsg = []

            self.hasBTrunk = False  #
            self.hasFTrunk = False  # 检测司机室后侧是否集卡
            self.distanceB = 999  # 返回给PLC的司机室后侧的距离数据
            self.distanceF = 999
            self.plcHB = 0  # 与PLC交互的信息编号

            # PLC
            self.plc_msg = ''
            self.spreaderSize = 1  # 吊具尺寸，作业箱尺寸
            self.lockOffset = self.config["lockOffset"]  # 集卡车尾到锁头的偏移距离，单位为厘米
            self.taskType = True  # 装箱（TRUE）或者卸箱（False）的任务类型
            self.workPosition = 0  # 作业地点 司机室后侧（2）或者司机室前侧（1）,不作业（0）
            self.needGuide = False  # 是否需要引导（外集卡需要引导(1)，内集卡不需要(0)
            self.container_in_out = 0  # 1:进箱指令，2：出箱指令  0：无效

            self.boxPos_B = 0  # 用来记录托板上箱子个数和位置情况：0：空箱 1：前箱有箱 2：后箱有箱 3：满箱
            self.boxPos_F = 0

            self.plcmessage_printcount = 0

            # 存储最近三次的距离数值，取平均值作为本次的结果
            self.B_queue = []
            self.F_queue = []

            self.lidarB_count = 0  # 司机室后侧扫描仪的心跳
            self.lidarF_count = 0

            self.Bcontainer_head_point = -14  # 司机室后侧有箱情况下的箱头X坐标
            self.Bcontainer_tail_point = -14  # 司机室后侧有箱情况下的箱尾X坐标
            self.Btrunk_tail_point = -14  # 司机室后侧车板尾X坐标

            self.Fcontainer_head_point = -14  # 司机室前侧有箱情况下的箱头X坐标
            self.Fcontainer_tail_point = -14  # 司机室前侧有箱情况下的箱尾X坐标
            self.Ftrunk_tail_point = -14  # 司机室前侧车板尾X坐标

            # self.BhasGate = self.config['BhasGate'] # 司机室后侧箱门是否朝向车尾
            # self.FhasGate = self.config['FhasGate'] # 司机室前侧箱门是否朝向车尾

        except Exception:
            traceback.print_exc()
            self.log.error(traceback.format_exc())

    def __del__(self):
        if self.lidarF:
            self.lidarF.joinThread()
            self.lidarF.disconnect()
        if self.lidarB:
            self.lidarB.joinThread()
            self.lidarB.disconnect()
        if self.PLC_socket:
            self.thread_switch = False
            self.t.join()
            self.PLC_socket.close()
        if self.ledF:
            self.ledF.serial_close()
        if self.ledB:
            self.ledB.serial_close()

    def initLidarB(self):
        self.lidarB = FM_TCP(
            self.config["BlidarIP"], self.config["BlidarPort"],
            self.config["BlidarCID"], self.config["BlidarName"]
        )
        self.lidarB.setOffsetX(self.config["BXoffset"])
        self.lidarB.setOffsetY(self.config["BYoffset"])
        self.lidarB.setOffsetTheta(self.config["BThetaoffset"])
        self.lidarB.setXReverse(self.config["BXReverse"])
        self.lidarB.setYReverse(self.config["BYReverse"])
        self.lidarB.connect()
        self.lidarB.sendStart()

    def initLidarF(self):
        # config the Front-side lidar
        self.lidarF = FM_TCP(
            self.config["FlidarIP"], self.config["FlidarPort"],
            self.config["FlidarCID"], self.config["FlidarName"]
        )
        self.lidarF.setOffsetX(self.config["FXoffset"])
        self.lidarF.setOffsetY(self.config["FYoffset"])
        self.lidarF.setOffsetTheta(self.config["FThetaoffset"])
        self.lidarF.setXReverse(self.config["FXReverse"])
        self.lidarF.setYReverse(self.config["FYReverse"])
        self.lidarF.connect()
        self.lidarF.sendStart()

    def refreshAllLidar(self):
        if self.config["guideChoice"] > 0:
            self.lidarF.setOffsetX(self.config["FXoffset"])
            self.lidarF.setOffsetY(self.config["FYoffset"])
            self.lidarF.setOffsetTheta(self.config["FThetaoffset"])
            self.lidarF.setXReverse(self.config["FXReverse"])
            self.lidarF.setYReverse(self.config["FYReverse"])
        if self.config["guideChoice"] < 0 or self.config["guideChoice"] > 1:
            self.lidarB.setOffsetX(self.config["BXoffset"])
            self.lidarB.setOffsetY(self.config["BYoffset"])
            self.lidarB.setOffsetTheta(self.config["BThetaoffset"])
            self.lidarB.setXReverse(self.config["BXReverse"])
            self.lidarB.setYReverse(self.config["BYReverse"])

    def initPLC(self):
        self.PLC_socket = socket(AF_INET, SOCK_STREAM)
        self.PLC_socket.connect((self.config["PLCIP"], self.config["PLCPort"]))

    def is_lidar_alive(self):
        # 扫描仪的有效性判断，对不正常心跳值进行计数判断
        lidarF_count = 0
        lidarB_count = 0
        if self.config["guideChoice"] > 0:
            lidarF_count = self.lidarF.getCount()
            if lidarF_count == self.lidarF_last_count:
                self.lidarF_count += 1
            else:
                self.lidarF_last_count = lidarF_count
                self.lidarF_count = 0
            if self.lidarF_count > self.config["aliveCount"]:
                self.log.error("{} Front Lidar heartbeat pauses and reconnects".format(self.config['deviceID']))
                self.lidarFState = False
        if self.config["guideChoice"] < 0 or self.config["guideChoice"] > 1:
            lidarB_count = self.lidarB.getCount()
            if lidarB_count == self.lidarB_last_count:
                self.lidarB_count += 1
            else:
                self.lidarB_last_count = lidarB_count
                self.lidarB_count = 0
            if self.lidarB_count > self.config["aliveCount"]:
                self.log.error("{} Behind Lidar heartbeat pauses and reconnects".format(self.config['deviceID']))
                self.lidarBState = False

    def loadCfgFile(self, filename='cpsconfig.json'):
        '''load config information for config-file '''
        with open(filename, 'r') as f:
            config = json.load(f)
        return config

    def sendPLC(self, message):
        '''send messages to PLC'''
        self.PLC_socket.sendall(message)

    def disconnected(self):
        self.thread_switch = False
        self.joinThread()
        self.PLC_socket.close()

    def startThread(self):
        self.thread_switch = True
        self.t.start()

    def joinThread(self):
        self.t.join()

    def _threadHandler(self):
        try:
            while self.thread_switch:
                recv = self.PLC_socket.recv(22)
                # print('recving from plc :',recv)
                self.spreaderSize = struct.unpack('>H', recv[6:8])[0]  # 1: 20英尺； 2: 40英尺； 3: 双20英尺；
                # 开锁：集卡进行卸箱任务
                self.taskType = not (struct.unpack('>H', recv[10:12])[0] >> 2 & 1)
                temp = struct.unpack('>H', recv[12:14])[0]
                if temp:
                    # 如果PLC传输了锁头到车尾的偏移量就采用PLC传输的配置，否则采用配置文件的锁头偏移量
                    self.lockOffset = temp
                self.workPosition = struct.unpack('>H', recv[16:18])[0]
                self.needGuide = struct.unpack('>H', recv[18:20])[0]
                self.container_in_out = struct.unpack('>h', recv[20:22])[0]  #

                self.plc_msg = \
                    "【箱子作业尺寸：{0}】1:20尺 2:40尺 3：双20尺\n" \
                    "【任务类型：{1}】\n" \
                    "【车尾锁头偏移：{2}】\n" \
                    "【作业位置：{3}】\n" \
                    "【是否引导：{4}】\n" \
                    "【进出箱】:{5}\n".format(
                        '20尺' if self.spreaderSize == 1 else '40尺' if self.spreaderSize == 2 else '双20尺'
                        if self.spreaderSize == 3 else "45尺",
                        '闭锁-装箱' if self.taskType else '开锁-卸箱', self.lockOffset,
                        '不作业' if not self.workPosition else '司机室后侧' if self.workPosition == 2 else '司机室前侧',
                        '需要引导' if self.needGuide else '不需要引导',
                        '进箱' if self.container_in_out == 1 else '出箱' if self.container_in_out == 2 else '待定')

        except Exception as e:
            print('线程出现故障，退出了！！', e)
            # self.disconnected()

    def dataLog(self, x, y, msg=''):
        if self.config['dataLog']:
            x = [round(ele, 3) for ele in x]
            y = [round(ele, 3) for ele in y]
            now = datetime.now().strftime('-%m-%d')
            fileName = 'dataLog' + now + '.txt'
            timeStamp = strftime("%Y-%m-%d %H:%M:%S", localtime())
            with open(fileName, 'a') as file:
                file.write(timeStamp + msg + '\n' + str(x) + '\n' + str(y) + '\n')

    def cutAll(self, x, y):
        '''
            初步切割：查找需要范围内的点，temp_x y用来画图显示 trunk_x y用来记录集卡点数据
        '''
        # 截取了高于地面上车体部分
        trunkTopHeight = 5
        trunk_x, trunk_y = [], []
        temp_x, temp_y = [], []
        for index in range(len(x)):
            if self.config["leftEdge"] < x[index] <= self.config["rightEdge"] and \
                    0 < y[index] < self.config["maxHeight"]:
                temp_x.append(x[index])
                temp_y.append(y[index])
                # 截取所有数据中的车体部分
                if self.config['minTrunkHeight'] < y[index] < trunkTopHeight:
                    trunk_x.append(x[index])
                    trunk_y.append(y[index])
        return temp_x, temp_y, trunk_x, trunk_y

    def get_chassis_outline(self, trunk_x, trunk_y):
        """
        获取集卡完整轮廓点集
        :return:
        """
        outline_x = []
        outline_y = []
        if len(trunk_x) < 2:
            return [], []
        seed = 0
        seed_index = 0
        threshold = self.config["outlineThreshold"]
        trunk_x, trunk_y = sortFirstElement(trunk_x, trunk_y)
        for i in trunk_x:
            if 0 <= i < 1:
                seed = i
                seed_index = trunk_x.index(i)
                break
        if seed:
            refer_x = seed
            refer_index = seed_index
            outline_x.append(seed)
            outline_y.append(trunk_y[refer_index])
            for i in range(seed_index - 1, 0, -1):
                if refer_x - trunk_x[i] > threshold:
                    break
                outline_x.insert(0, trunk_x[i])
                outline_y.insert(0, trunk_y[i])
                refer_x = trunk_x[i]
                refer_index = i
            refer_x = seed
            refer_index = seed_index
            for i in range(seed_index + 1, len(trunk_x) - 1):
                if trunk_x[i] - refer_x > threshold:
                    break
                outline_x.append(trunk_x[i])
                outline_y.append(trunk_y[i])
                refer_x = trunk_x[i]
                refer_index = i
        return outline_x, outline_y

    def findTailPoint(self, trunk_x, trunk_y):
        """
        return 车尾x坐标，箱尾x坐标，箱头X坐标
        """
        box_tail_point = -15
        box_head_point = -15
        trunk_tail_point = -15
        boxPos = 0
        trunk_x, trunk_y = sortFirstElement(trunk_x, trunk_y)
        box_x = []
        box_y = []
        boxHeight = 5  # 载箱后的箱子高度不会超过5米

        # 对尾部数据进行滤波处理
        if len(trunk_x) > 18:
            for i in range(5):
                if (trunk_x[1] - trunk_x[0]) > self.config["threshold"]:
                    del trunk_x[0]
                    del trunk_y[0]
                else:
                    break
        if not trunk_x or len(trunk_x) < 20:
            return trunk_tail_point, box_tail_point, box_head_point, boxPos

        if len(trunk_x) > 2:
            tailpoint_Xsub = trunk_x[1] - trunk_x[0]
        else:
            tailpoint_Xsub = 0

        if len(trunk_y) > 10 and 0.035 < tailpoint_Xsub <= self.config["threshold"]:
            trunk_tail_point = (trunk_x[1] + trunk_x[0]) / 2
        elif len(trunk_y) > 10 and tailpoint_Xsub <= 0.035:
            trunk_tail_point = trunk_x[0]

        # 集卡车长度值
        trunkLength = trunk_x[-1] - trunk_tail_point

        # 判断车体是否完全进入检测范围
        if trunk_tail_point > self.config["leftEdge"] + 1 and trunkLength > 6:
            # 车体完全进入区间 判断有没有放箱
            for index in range(len(trunk_x)):
                # 记录托板上面箱子的值
                if (self.config['minBoxHeight'] < trunk_y[index] < boxHeight) and \
                        ((trunk_tail_point - 0.1) <= trunk_x[index] < trunk_tail_point + 12.5):
                    box_x.append(trunk_x[index])
                    box_y.append(trunk_y[index])

            if len(box_x) < 15:
                box_x = []  # 点数太少认为是跳点干扰引起
                box_y = []

            if len(box_x) < 15:
                # 空箱状态：
                boxPos = 0
            else:
                # 滤波处理
                if len(box_x) > 18:
                    for i in range(8):
                        if box_x[1] - box_x[0] > self.config["threshold"]:
                            del box_x[0]
                            del box_y[0]
                        else:
                            break
                if len(box_x) > 50:
                    for i in range(8):
                        if abs(box_x[-2] - box_x[-1]) > self.config["threshold"]:
                            del box_x[-1]
                            del box_y[-1]
                        else:
                            break

                box_tail_Xsub = box_x[1] - box_x[0]
                if 0.035 < box_tail_Xsub <= self.config["threshold"]:
                    box_tail = (box_x[0] + box_x[1]) / 2
                else:
                    box_tail = box_x[0]

                if box_tail < trunk_tail_point + 3:
                    # 后箱位有箱
                    boxPos += 2
                if max(box_x) > trunk_tail_point + 7:
                    # 前箱位有箱
                    boxPos += 1

                box_head_point = box_x[-1]
                box_tail_point = box_tail

        return trunk_tail_point, box_tail_point, box_head_point, boxPos

    def plotFB(self, x1, y1, x2, y2, work_msg_B, work_msg_F):
        if self.config['plotSwitch']:
            self.plt.clf()
            self.plt.axis([self.config['leftEdge'], self.config['rightEdge'], -0.5, 19.6])
            # 画两条地平线
            if self.config["guideChoice"] < 0 or self.config["guideChoice"] > 1:
                self.plt.plot([self.config['leftEdge'], self.config['rightEdge']], [0, 0], '-')
                self.plt.plot(x1, y1, 'b.')  # B
            if self.config["guideChoice"] > 0:
                self.plt.plot([self.config['leftEdge'], self.config['rightEdge']], [10, 10], '-')
                y2 = [ele + 10 for ele in y2]
                self.plt.plot(x2, y2, 'g.')  # F
            # 在xlabel 上标注当前的工况
            if self.config["guideChoice"] < 0:
                self.plt.xlabel(work_msg_B, fontproperties='SimHei', fontsize=13)
                self.plt.title(self.config["deviceID"] + ' Behind:{}cm'.format(self.distanceB))
            elif self.config["guideChoice"] > 1:
                self.plt.xlabel(work_msg_F, fontproperties='SimHei', fontsize=13)
            else:
                self.plt.xlabel(work_msg_F + ' / ' + work_msg_B, fontproperties='SimHei', fontsize=13)
                # title 标注距离值
                self.plt.title(self.config["deviceID"] + ' Front :{}cm / Behind:{}cm'.format(
                    self.distanceF, self.distanceB))

            # 画出中心线
            self.plt.plot([0, 0], [-1, 40], '-')

            # 画出2个20尺辅助车厢线，方便调试
            if self.hasBTrunk and abs(self.distanceB) < 500:
                # 注意画图的单位是米，传来的参数单位是厘米
                Bx = self.Btrunk_tail_point
                x = []
                assistLineM_x = [Bx, Bx + self.ft20Len * 0.5 / 100, Bx + self.ft20Len / 100,
                                 Bx + self.ft20Len * 1.5 / 100,
                                 Bx + self.ft40Len / 100]
                temp = [[temp for ele in range(11)] for temp in assistLineM_x]
                for ele in temp:
                    x += ele
                y = [1 + ele * 0.5 for ele in range(1, 12, 1)] * 5  # 11 个点
                self.plt.plot(x, y, 'r.', name="司机室后侧")
                # 在图中标明车尾，箱尾，箱头三点的坐标点
                self.plt.text(self.Btrunk_tail_point - 2, 7, '车尾{}'.format(
                    round(self.Btrunk_tail_point, 3)),
                              fontsize=13, color='purple', fontproperties='SimHei')
                if self.boxPos_B:
                    self.plt.text(self.Bcontainer_tail_point - 2, 8, '箱尾{}'.format(
                        round(self.Bcontainer_tail_point, 3)),
                                  fontsize=13, color='purple', fontproperties='SimHei')
                    self.plt.text(self.Bcontainer_head_point - 2, 8, '箱头{}'.format(
                        round(self.Bcontainer_head_point, 3)),
                                  fontsize=13, color='purple', fontproperties='SimHei')

            if self.hasFTrunk and abs(self.distanceF) < 500:
                # 注意画图的单位是米，传来的参数单位是厘米
                Bx = self.Ftrunk_tail_point
                x = []
                assistLineM_x = [Bx, Bx + self.ft20Len * 0.5 / 100, Bx + self.ft20Len / 100,
                                 Bx + self.ft20Len * 1.5 / 100,
                                 Bx + self.ft40Len / 100]
                temp = [[temp for ele in range(11)] for temp in assistLineM_x]
                for ele in temp:
                    x += ele
                y = [10 + ele * 0.5 for ele in range(1, 12, 1)] * 5  # 11 个点
                self.plt.plot(x, y, 'r.', "司机室前侧")
                # 在图中标明车尾，箱尾，箱头三点的坐标点
                self.plt.text(self.Ftrunk_tail_point - 2, 15, '车尾{}'.format(round(self.Ftrunk_tail_point, 3)),
                              fontsize=13, color='orange', fontproperties='SimHei')
                if self.boxPos_F:
                    # 有箱时，标注箱尾和箱头的坐标点
                    self.plt.text(self.Fcontainer_tail_point - 2, 16, '箱尾{}'.format(
                        round(self.Fcontainer_tail_point, 3)),
                                  fontsize=13, color='orange', fontproperties='SimHei')
                    self.plt.text(self.Fcontainer_head_point - 2, 16, '箱头{}'.format(
                        round(self.Fcontainer_head_point, 3)),
                                  fontsize=13, color='orange', fontproperties='SimHei')

            self.plt.draw()
            self.plt.pause(0.001)
        else:
            self.plt.close()
            sleep(0.1)

    def getCPSControl(self):
        self.plcHB += 1
        if self.plcHB > 65534:
            self.plcHB = 0
        sendInfo = b''
        sendInfo += struct.pack('>H', self.plcHB)

        if self.config["guideChoice"] > 1:
            # 司机室前后两侧装车道均引导
            # 司机室后侧扫描仪状态
            num1 = 0
            num1 = num1 | self.lidarBState
            num1 = num1 | (self.lidarB.getDirtyState() << 1)
            num1 = num1 | ((1 << 2) if abs(self.distanceB) < self.config["guideDoneThreshold"] else (0 << 2))
            num1 = num1 | (self.hasBTrunk << 3)
            num1 = num1 | (0 << 7)
            sendInfo += struct.pack(">B", num1)

            # 司机室前侧扫描仪状态
            num2 = 0
            num2 = num2 | self.lidarFState
            num2 = num2 | (self.lidarF.getDirtyState() << 1)
            num2 = num2 | ((1 << 2) if abs(self.distanceF) < self.config["guideDoneThreshold"] else (0 << 2))
            num2 = num2 | (0 << 7)
            sendInfo += struct.pack(">B", num2)

            sendInfo += struct.pack('>h', self.distanceF)
            sendInfo += struct.pack('>h', self.distanceB)

        elif self.config["guideChoice"] < 0:
            # 仅司机室后侧装车道引导
            # 司机室后侧扫描仪状态
            num1 = 0
            num1 = num1 | self.lidarBState
            num1 = num1 | (self.lidarB.getDirtyState() << 1)
            num1 = num1 | ((1 << 2) if abs(self.distanceB) < self.config["guideDoneThreshold"] else (0 << 2))
            num1 = num1 | (self.hasBTrunk << 3)
            num1 = num1 | (0 << 7)
            sendInfo += struct.pack(">B", num1)

            # 司机室前侧扫描仪状态,默认状态
            num2 = 0b10100000
            sendInfo += struct.pack(">B", num2)

            sendInfo += struct.pack('>h', self.distanceF)
            sendInfo += struct.pack('>h', self.distanceB)
        else:
            # 仅司机室前侧装车道引导
            # 司机室后侧扫描仪状态, 默认状态
            num1 = 0b10100000
            sendInfo += struct.pack(">B", num1)

            # 司机室前侧扫描仪状态
            num2 = 0
            num2 = num2 | self.lidarFState
            num2 = num2 | (self.lidarF.getDirtyState() << 1)
            num2 = num2 | ((1 << 2) if abs(self.distanceF) < self.config["guideDoneThreshold"] else (0 << 2))
            num2 = num2 | (0 << 7)
            sendInfo += struct.pack(">B", num2)

            sendInfo += struct.pack('>h', self.distanceF)
            sendInfo += struct.pack('>h', self.distanceB)
        if not self.plcState:
            sendInfo += struct.pack('>H', 5)
        else:
            sendInfo += struct.pack('>H', 0)
        return sendInfo

    def cal_distance_B(self, x, y):
        # 计算司机室后侧的引导距离值
        work_msg = '司机室后侧装车道：'
        if len(x) > 20 and len([i for i in y if i > 0.3]) > 20:  # 有集卡出现
            hasTrunk = True
            x, y = self.get_chassis_outline(x, y)
            self.Btrunk_tail_point, self.Bcontainer_tail_point, \
            self.Bcontainer_head_point, self.boxPos_B = self.findTailPoint(x, y)
            # test todo
            # 如果箱门朝向车尾，则修正箱门到边缘差值引起的误差
            # if self.BhasGate:
            #     self.Bcontainer_tail_point -= 0.05
            #     work_msg += ' 有箱门 '
            self.Bcontainer_tail_point -= 0.025

            if self.workPosition and self.Btrunk_tail_point > -12:
                # 尺寸: 20英尺； 2: 40英尺； 3: 双20英尺；4: 45尺
                if self.spreaderSize == 2:  # 40尺作业情况
                    if self.boxPos_B == 0:  # 空箱
                        work_msg += '空板 放 40尺箱'
                        distance = -int(self.Btrunk_tail_point * 100 + self.ft40Len * 0.5 + self.lockOffset) \
                                   - self.config["Behind_40ft_load_offset"]
                    else:  # 满箱
                        work_msg += '满箱 抓 40尺箱'
                        distance = -int(self.Bcontainer_tail_point * 100 + self.ft40Len * 0.5) + \
                                   -self.config["Behind_40ft_unload_offset"]

                    distance += self.config['Behind_40ft_offset']  # 吊具中心偏移量
                elif self.spreaderSize == 4:
                    # 45尺
                    if not self.boxPos_B:
                        work_msg += "空板 放 45尺箱"
                        distance = -int(self.Btrunk_tail_point * 100 + self.ft45Len * 0.5 + self.lockOffset) \
                                   - self.config["Behind_45ft_load_offset"]
                    else:
                        work_msg = "满箱 抓 45尺箱"
                        distance = -int(self.Bcontainer_tail_point * 100 + self.ft45Len * 0.5) + \
                                   -self.config["Behind_45ft_unload_offset"]
                    distance += self.config["Behind_45ft_offset"]
                elif self.boxPos_B == 0:
                    # 20 尺 空板 前箱
                    if self.Btrunk_tail_point * 100 < -self.ft20Len:
                        work_msg += '空板 前箱位 放 20尺箱'
                        distance = -int(self.Btrunk_tail_point * 100 + self.ft20Len * 1.5 + self.lockOffset +
                                        self.gapLen) - self.config["Behind_20ft_head_empty_offset"]
                    else:
                        # 后箱位置
                        work_msg += '空板 后箱位 放 20尺箱'
                        distance = -int(self.Btrunk_tail_point * 100 + self.ft20Len * 0.5 + self.lockOffset) + \
                                   -self.config["Behind_20ft_back_empty_offset"]

                    distance += self.config['Behind_20ft_offset']

                elif self.boxPos_B == 1:  # 20 有前箱
                    if self.Btrunk_tail_point * 100 < -self.ft20Len:
                        work_msg += '有前箱 前箱位 抓 20尺箱'
                        distance = -int(self.Bcontainer_tail_point * 100 + self.ft20Len * 0.5) \
                                   - self.config['Behind_20ft_forward_unload_offset']

                    else:
                        # 采用前箱尾坐标和车尾坐标分别计算箱位中心偏移量然后取均值，消除误差
                        work_msg += '有前箱 后箱位 放 20尺箱'
                        distance = (-int(self.Bcontainer_tail_point * 100 - self.ft20Len * 0.5 - self.gapLen) -
                                    (self.ft20Len * 0.5 + self.Btrunk_tail_point * 100)) / 2 - self.config[
                                       "Behind_20ft_forward_load_offset"]

                    distance += self.config['Behind_20ft_offset']

                elif self.boxPos_B == 2:  # 20尺有后箱
                    if self.Btrunk_tail_point * 100 >= -self.ft20Len:
                        work_msg += '有后箱 后箱位 抓 20尺箱'
                        distance = -int(self.Bcontainer_tail_point * 100 + self.ft20Len * 0.5) + \
                                   -self.config["Behind_20ft_backward_unload_offset"]
                    else:
                        work_msg += '有后箱 前箱位 放 20尺箱'
                        distance = -int(self.Bcontainer_tail_point * 100 + self.ft20Len * 1.5 + self.gapLen) - \
                                   self.config["Behind_20ft_backward_load_offset"]

                    distance += self.config['Behind_20ft_offset']

                else:
                    if self.Btrunk_tail_point * 100 < -self.ft20Len:
                        work_msg += '双20尺满箱 前箱位 抓箱'
                        # 使用箱头坐标和箱尾坐标分别计算箱位中心偏移量，减少误差
                        distance = (-int(self.Bcontainer_tail_point * 100 + self.ft20Len * 1.5 + self.gapLen) +
                                    self.ft20Len * 0.5 - self.Bcontainer_head_point * 100) / 2 + \
                                   -self.config["Behind_20ft_full_unload_head_offset"]
                    else:
                        work_msg += '双20尺满箱 后箱位 抓箱'
                        distance = -int(self.Bcontainer_tail_point * 100 + self.ft20Len * 0.5) + \
                                   -self.config["Behind_20ft_full_unload_back_offset"]

                    distance += self.config['Behind_20ft_offset']
            else:
                # 不需要引导的情况
                work_msg += "不需要引导"
                distance = 999
        else:
            work_msg += "集卡未出现"
            hasTrunk = False
            distance = 999

        return int(distance), work_msg, hasTrunk

    def cal_distance_F(self, x, y):
        work_msg = '司机室前侧装车道：'
        if len(x) > 20 and len([i for i in y if i > 0.3]) > 20:  # 有集卡出现
            hasTrunk = True
            x, y = self.get_chassis_outline(x, y)
            self.Ftrunk_tail_point, self.Fcontainer_tail_point, \
            self.Fcontainer_head_point, self.boxPos_F = self.findTailPoint(x, y)

            # test todo
            # 如果箱门朝向车尾，则修正箱门到边缘差值引起的误差
            # if self.FhasGate:
            #     self.Fcontainer_tail_point -= 0.05
            #     work_msg += ' 有箱门 '
            # 补偿箱门朝向车尾引起计算的引导数据偏差
            self.Fcontainer_tail_point -= 0.025

            if self.workPosition and self.Ftrunk_tail_point > -12:
                # 尺寸: 20英尺； 2: 40英尺； 3: 双20英尺；
                if self.spreaderSize == 2:  # 40尺作业情况
                    if self.boxPos_F == 0:  # 空箱
                        work_msg += '空板 放 40尺箱'
                        distance = -int(self.Ftrunk_tail_point * 100 + self.ft40Len * 0.5 + self.lockOffset) \
                                   - self.config["Front_40ft_load_offset"]
                    else:  # 满箱
                        work_msg += '有40尺箱 抓 40尺箱'
                        distance = -int(self.Fcontainer_tail_point * 100 + self.ft40Len * 0.5) + \
                                   -self.config["Front_40ft_unload_offset"]

                    distance += self.config['Front_40ft_offset']  # 吊具中心偏移量
                elif self.spreaderSize == 4:
                    # 45尺
                    if not self.boxPos_F:
                        work_msg += "空板 放 45尺箱"
                        distance = -int(self.Ftrunk_tail_point * 100 + self.ft45Len * 0.5 + self.lockOffset) \
                                   - self.config["Front_45ft_load_offset"]
                    else:
                        work_msg = "满箱 抓 45尺箱"
                        distance = -int(self.Fcontainer_tail_point * 100 + self.ft45Len * 0.5) + \
                                   -self.config["Front_45ft_unload_offset"]
                    distance += self.config["Front_45ft_offset"]
                elif self.boxPos_F == 0:
                    # 20 尺 空板 前箱
                    if self.Ftrunk_tail_point * 100 < -self.ft20Len:
                        work_msg += '空板 前箱位 放 20尺箱'
                        distance = -int(
                            self.Ftrunk_tail_point * 100 + self.ft20Len * 1.5 + self.lockOffset + self.gapLen) + \
                                   -self.config["Front_20ft_head_empty_offset"]
                    else:
                        # 后箱位置
                        work_msg += '空板 后箱位 放 20尺箱'
                        distance = -int(self.Ftrunk_tail_point * 100 + self.ft20Len * 0.5 + self.lockOffset) + \
                                   -self.config["Front_20ft_back_empty_offset"]

                    distance += self.config['Front_20ft_offset']  # 20尺吊具中心偏移量

                elif self.boxPos_F == 1:  # 20 有前箱
                    if self.Ftrunk_tail_point * 100 < -self.ft20Len:
                        work_msg += '有前箱 前箱位 抓 20尺箱'
                        distance = -int(self.Fcontainer_tail_point * 100 + self.ft20Len * 0.5) \
                                   - self.config['Front_20ft_forward_unload_offset']
                    else:
                        work_msg += '有前箱 后箱位 放 20尺箱'
                        # 使用前箱头坐标和车尾坐标分别换算箱位中心偏移量并取均值减少误差值
                        distance = (-int(self.Fcontainer_tail_point * 100 - self.ft20Len * 0.5 - self.gapLen) - (
                                self.ft20Len * 0.5 + self.Ftrunk_tail_point * 100)) / 2 + - self.config[
                            "Front_20ft_forward_load_offset"]

                    distance += self.config['Front_20ft_offset']

                elif self.boxPos_F == 2:  # 20尺有后箱
                    if self.Ftrunk_tail_point * 100 >= -self.ft20Len:
                        work_msg += '有后箱 后箱位 抓 20尺箱'
                        distance = ((-int(self.Fcontainer_tail_point * 100 + self.ft20Len * 0.5)) +
                                    self.ft20Len * 0.5 - self.Fcontainer_head_point * 100) / 2 + \
                                   -self.config["Front_20ft_backward_unload_offset"]
                    else:
                        work_msg += '有后箱 前箱位 放 20尺箱'
                        distance = -int(self.Fcontainer_tail_point * 100 + self.ft20Len * 1.5 + self.gapLen) + \
                                   -self.config["Front_20ft_backward_load_offset"]

                    distance += self.config['Front_20ft_offset']

                else:
                    # 20 尺满箱，作业前箱
                    if self.Ftrunk_tail_point * 100 < -self.ft20Len:
                        work_msg += '双20尺满箱 前箱位 抓箱'
                        distance = (-int(self.Fcontainer_tail_point * 100 + self.ft20Len * 1.5 + self.gapLen) +
                                    self.ft20Len * 0.5 - self.Fcontainer_head_point * 100) / 2 + \
                                   -self.config["Front_20ft_full_unload_head_offset"]
                    else:
                        # 作业后箱
                        work_msg += '双20尺满箱 后箱位 抓箱'
                        distance = -int(self.Fcontainer_tail_point * 100 + self.ft20Len * 0.5) + \
                                   -self.config["Front_20ft_full_unload_back_offset"]

                    distance += self.config['Front_20ft_offset']

            else:
                # 不需要引导的情况
                work_msg += "不需要引导"
                distance = 999
        else:
            work_msg += "集卡未出现"
            hasTrunk = False
            distance = 999

        return int(distance), work_msg, hasTrunk

    def distance_filter(self):
        if self.config["guideChoice"] > 0:
            self.F_queue.append(self.distanceF)
            # 只存储最近三次数据
            if len(self.F_queue) > 3:
                del self.F_queue[0]
            self.distanceF = int(np.mean(self.F_queue))
        if self.config["guideChoice"] < 0 or self.config["guideChoice"] > 1:
            self.B_queue.append(self.distanceB)
            if len(self.B_queue) > 3:
                del self.B_queue[0]
            self.distanceB = int(np.mean(self.B_queue))

    def start(self):
        if self.config['PLCSwitch']:
            self.startThread()  # 开启线程，接收PLC发送的数据，并解包

        count = 0
        error_count = 0
        time_count = 0
        work_msg_B = ''
        work_msg_F = ''

        while True:
            starttime = time()
            self.plcmessage_printcount += 1
            try:
                msg = self.getCPSControl()
                self.sendPLC(msg)
                try:
                    # print("------------counting: %d--------------" % count)
                    if count < self.config["scanConfigRotation"]:
                        count += 1
                    else:
                        count = 0
                        self.config = self.loadCfgFile()
                        self.refreshAllLidar()

                        if self.config["guideChoice"] > 0:
                            try:
                                self.lidarF.sendHeartBeat()
                            except Exception:
                                self.lidarFState = False
                            else:
                                self.lidarFState = True
                        if self.config["guideChoice"] < 0 or self.config["guideChoice"] > 1:
                            try:
                                self.lidarB.sendHeartBeat()
                            except Exception:
                                self.lidarBState = False
                            else:
                                self.lidarBState = True

                    self.lidarBState = True
                    self.lidarFState = True
                    self.is_lidar_alive()
                    Ftrunk_x = 0
                    Ftrunk_y = 0
                    Btrunk_x = 0
                    Btrunk_y = 0
                    plot_x1 = []
                    plot_y1 = []
                    plot_x2 = []
                    plot_y2 = []
                    if self.config["guideChoice"] > 0:
                        x2, y2 = self.lidarF.getXY()
                        plot_x2, plot_y2, Ftrunk_x, Ftrunk_y = self.cutAll(x2, y2)
                    if self.config["guideChoice"] < 0 or self.config["guideChoice"] > 1:
                        x1, y1 = self.lidarB.getXY()
                        plot_x1, plot_y1, Btrunk_x, Btrunk_y = self.cutAll(x1, y1)
                    # PLC从TOS读取的锁头偏移量在有效数据范围内时采用，否则采用默认值
                    if self.lockOffset < 0 or self.lockOffset > 500:
                        self.lockOffset = 0

                    if self.needGuide:
                        if self.config["guideChoice"] > 0:
                            self.distanceF, work_msg_F, self.hasFTrunk = self.cal_distance_F(Ftrunk_x, Ftrunk_y)
                        if self.config["guideChoice"] < 0 or self.config["guideChoice"] > 1:
                            self.distanceB, work_msg_B, self.hasBTrunk = self.cal_distance_B(Btrunk_x, Btrunk_y)

                    else:
                        self.distanceB = 999
                        self.distanceF = 999

                    # 取最近三次的平均值作为本次的引导数据
                    self.distance_filter()
                    # 根据配置文件的开关选择是否画图显示
                    if self.config["plotSwitch"]:
                        self.plotFB(plot_x1, plot_y1, plot_x2, plot_y2, work_msg_B, work_msg_F)
                    else:
                        sleep(0.1)
                        self.plt.close()

                    # 集卡进入小范围内即开始记录点云数据
                    if self.config["logSwitch"]:
                        if self.plcmessage_printcount > 10 and abs(self.distanceB) < 200:
                            self.dataLog(plot_x2, plot_y2)
                        if self.plcmessage_printcount > 10 and abs(self.distanceF) < 200:
                            self.dataLog(plot_x1, plot_y1)

                    # 回传给PLC的信息
                    if self.config['PLCSwitch']:
                        msg = self.getCPSControl()
                        try:
                            self.sendPLC(msg)
                        except Exception:
                            self.disconnected()
                            self.plcState = False
                            self.plc_reconnect_count += 1
                            sleep(0.5)
                        else:
                            self.plc_reconnect_count = 0
                            self.plcState = True
                    if self.config["guideChoice"] > 0:
                        if self.distanceF == 999:
                            if self.ledF:
                                self.ledF.stand_by()
                        else:
                            if self.ledF:
                                self.ledF.set_cmd(self.distanceF)
                    if self.config["guideChoice"] < 0 or self.config["guideChoice"] > 1:
                        if self.distanceB == 999:
                            if self.ledB:
                                self.ledB.stand_by()
                        else:
                            if self.ledB:
                                self.ledB.set_cmd(self.distanceB)
                    if not self.plcState:
                        try:
                            self.initPLC()
                        except Exception:
                            m = "{}:PLC disconnected {}".format(
                                self.config['deviceID'], traceback.format_exc())
                            self.log.error(m)
                            print(m)
                            self.plcState = False
                            self.plc_reconnect_count += 1
                        else:
                            if self.PLC_socket:
                                self.startThread()
                                self.plcState = True
                                self.plc_reconnect_count = 0

                    if not self.lidarBState:
                        try:
                            self.lidarB.disconnect()
                            self.lidarB.joinThread()
                            self.initLidarB()
                        except Exception:
                            self.lidarBState = False
                            m = "{}:CPS Behind Lidar disconnected {}".format(
                                self.config['deviceID'], traceback.format_exc())
                            print(m)
                            self.log.error(m)
                            self.lidarB_reconnect_count += 1
                        else:
                            self.lidarBState = True
                            self.lidarB_reconnect_count = 0

                    if not self.lidarFState:
                        try:
                            self.lidarF.disconnect()
                            self.lidarF.joinThread()
                            self.initLidarF()
                        except Exception:
                            self.lidarFState = False
                            m = "{}:CPS Front lidar disconnected {}".format(
                                self.config['deviceID'], traceback.format_exc()
                            )
                            self.log.error(m)
                            self.lidarF_reconnect_count += 1
                        else:
                            self.lidarFState = True
                            self.lidarF_reconnect_count = 0

                    # if the reconnecting count is bigger than 15  then exit
                    if self.lidarF_reconnect_count > 10 or self.lidarB_reconnect_count > 10 or \
                            self.plc_reconnect_count > 10:
                        self.log.error('plc or lidar reconnectint count is more than 15 times and reboot')
                        self.__del__()
                        sys.exit(0)

                    # print key message for user
                    if self.plcmessage_printcount > 10:
                        print(self.plc_msg)
                        loop_time = time() - starttime
                        msg_all = STDOUT_FMT.format(
                            self.config['deviceID'], work_msg_B + " / " + work_msg_F,
                            round(self.Btrunk_tail_point, 3), round(self.Bcontainer_tail_point, 3),
                            round(self.Bcontainer_head_point, 3), self.distanceB,
                            round(self.Ftrunk_tail_point, 3), round(self.Fcontainer_tail_point, 3),
                            round(self.Fcontainer_head_point, 3), self.distanceF, round(loop_time, 3))
                        print(msg_all)
                        # 出现连续15次以上的循环超时，则重启程序
                        if loop_time > 2:
                            time_count += 1
                            print('loop time over flow:{}'.format(loop_time))
                            self.log.warning('loop time over flow:{}'.format(loop_time))
                        else:
                            time_count = 0
                        if time_count > 15:
                            self.log.warning('loop time count is more than 15 times . exit now')
                            self.__del__()
                            sys.exit(0)
                        self.plcmessage_printcount = 0
                except KeyboardInterrupt:
                    self.__del__()
                    sys.exit(-1)
            except Exception as e:
                error_count += 1
                self.log.warning("catched an exception " + traceback.format_exc())
                traceback.print_exc()
                sleep(0.3)
            finally:
                if error_count > 10:
                    self.log.error('error count more than 15 times then exit and reboot')
                    self.__del__()
                    sys.exit(-1)
                # 喂狗
                try:
                    FD.feed()
                except Exception:
                    #  看门狗关闭，程序退出
                    self.__del__()
                    sys.exit(-1)


if __name__ == '__main__':
    try:
        obj = CPSSystem()
        obj.start()
    except Exception as e:
        traceback.format_exc()
        sleep(1)
        sys.exit(-1)
