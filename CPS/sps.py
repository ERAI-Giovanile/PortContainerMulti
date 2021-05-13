# Author:Amee
# last time : 2019-10-29 10:03
# 防打保龄总库文件
"""
todo
确认变量有没有清空处理
确认变量有没有在PLC出赋值处理
减小扫描仪的扫描范围，较少扫描仪附近的干扰点。

规定：
和PLC通讯中断超过2s，报SPS通讯故障
FM_1 : 匹配山侧的扫描仪  FM_2 : 匹配水侧的扫描仪
"""
import sys
import json
import traceback
from struct import pack
from datetime import datetime
from time import time, strftime, localtime, sleep

import matplotlib.pyplot as plt
import numpy as np

from logger_util import UniversalLogger
from FreeMeasure401R import FM_TCP
from FeedDog import FeedWatchDog
from plc_tcp import PLC_TCP
from sick511 import Sick511

STDOUT_FMT = """+-------------------------------{deviceID}------------------------------------+
司机室前后扫描仪捕获的高亮点数分别为: {RSSIF_num} / {RSSIB_num}
两侧扫描仪根据高亮点计算的吊具高度分别为: {RSSI_HF} / {RSSI_HB}
两侧扫描仪普通冗余计算的吊具高度分别为: {spreaderHF} / {spreaderHB}
综合运算后最终吊具高度为: {spreaderH: .3f}; PLC读取的吊具高度数值为: {PLC_spreaderH: .3f}(差值:{delta: .3f})
扫描仪坐标系中，小车位置为:{trolleyLocation: .3f}; PLC读取的小车位置为: {PLC_trolleyL: .3f}
    
当前是否载箱: {isLoad}; 是否跨贝: {isOverShell}; 小车运动方向: {moveDir}; 吊具起升状态: {spreaderState}
吊具额定减速时间: {slowTime}; 当前速度: {speed}%; 吊具下集装箱高度: {conH: .3f}m
当前贝每排箱高分别为: {everyH}
当前位置小车方向是否有障碍物: {hasBanner}; 前方障碍物距离: {bannerDis1: .3f}m; 后方障碍物距离:{bannerDis2: .3f}m    
是否防砸箱减速: {inDanger}; 吊具高度: {spreaderH: .3f}; 下方箱高: {thisH: .3f}m; 吊具速度: {spreaderSpeed}%; 减速距离: {lowDis: .3f}m                               
+-----------------------------------------------------------------------------------------+"""


class SPSSytem(object):
    def __init__(self):
        # 司机室前的扫描仪
        self.lidarF = None
        # 司机室后的扫描仪
        self.lidarB = None
        # 从扫描仪接收的坐标数据
        self.lidarF_x = None
        self.lidarF_y = None
        self.lidarB_x = None
        self.lidarB_y = None
        self.lidar_all_x = None
        self.lidar_all_y = None
        self.plc = None
        self.plc_spreader_height = None
        # 开始看门狗的程序初始化
        self.FD = FeedWatchDog(8889)
        # # 第一次喂狗
        self.FD.feed()
        self.log = UniversalLogger("SPS").logger
        self.plc_state = True  # plc状态表示标志，True：在线 False:掉线
        self.lidarF_state = True  # 司机室前扫描仪的装填标志
        self.lidarB_state = True  # 司机室后扫描仪的状态标志，True：在线，FALSE：掉线
        self.spreader_error_count = 0
        # 程序向调试软件发送PLC信息的标识，确保每次连接后调用
        self.plc_flag = False
        self.spreader_cache = 0
        self.chassis_lift_risk = b'\x00'  # 集卡被吊起风险，b'\x00'无风险，b'\x01'山侧集卡被吊起风险，b'\x02'水侧集卡被吊起风险

        self.lidarF_count = 0  # 司机室前扫描仪心跳重复计数
        self.lidarB_count = 0  # 司机室后扫描仪心跳重复计数
        self.lidarF_last_count = 0  # 程序历史记录的心跳序号
        self.lidarB_last_count = 0
        self.trolley_location = 0  # 小车位置，单位为米
        self.last_trolley_location = 0  # 记录上一次的端梁位置信息
        self.trolley_location_plc = 0  # plc 发送过来的小车位置
        self.is_lidarF_dirty = False  # 司机室前扫描仪镜面污染或者有遮挡
        self.is_lidarB_dirty = False  # 司机室后扫描仪镜面污染或者有遮挡

        self.spreader_height = 0  # 通过RSSI和XY计算吊具互相验证得出的吊具高度数值，用于PLC校验
        self.first_spreader_height = 0  # 由RSSI高亮点计算出来的吊具高度数据，单位为米
        self.second_spreader_height = 0  # 由吊具xy点计算出来的吊具高度数据，单位为米
        self.last_spreader_height = 0  # 记录上一次的吊具高度数据，用于缺失情况，单位为米

        self.real_row_height_send = [0]  # 记录每一排的箱子高度数据，单位为mm
        self.logPrefix = "error"  # 用于收集xy数据的文件的前缀

        # 用于小车方向防撞的变量
        self.trolley_direction = 0  # 小车的运动方向 1 向司机室前方向运动
        self.is_spreader_load = False  # 吊具是否有载箱
        self.head_barrier = 50  # 小车前方的障碍物与小车的距离，默认为最大值50米
        self.back_barrier = 50  # 小车后方的障碍物与小车的距离，默认为最大值50米

        # 吊具下放过程中的防砸箱
        self.is_spreader_down = False
        self.rated_speed = 0  # 额定速度 单位m/min
        self.rated_time = 0  # 额定减速时间 s
        self.current_speed = 0  # 当前吊具下降速度百分比%
        self.current_row = 0  # 小车当前的排位
        self.spreader_warning = False  # 吊具砸箱报警标志，当刹车距离达到给定值时，给限速信号
        self.is_rmg_moving = False  # 大车是否跨呗信号

        # 用于画图显示高亮点的位置
        self.rssi_temp_x = []
        self.rssi_temp_y = []

        self.left_limit = 0  # 当前小车位置的左边界
        self.right_limit = 0  # 当小车前位置的右边界

        self.send_count = 0  # plc 发送心跳计数

        self.warning_flag_F = False  # 小车前方障碍物提示标志
        self.warning_flag_B = False  # 小车后方障碍物提示标志

        self.container_size = 0  # 当前载箱的箱高尺寸
        self.scan_counter = 0  # 循环计数

        self.plc_reconnect_count = 0  # plc 掉线重连次数计数
        self.lidarF_reconnect_count = 0  # 司机室前扫描仪的重连次数计数
        self.lidarB_reconnect_count = 0  # 司机室后侧的扫描仪的重连次数计数
        self.plc_msg_count = 0  # 没有收到PLC数据次数计数

        self.spreader_width_load = []
        self.spreader_width_unload = []
        self.spreader_width_load_mean = 0
        self.spreader_width_unload_mean = 0
        self.statics_load_flag = True
        self.statics_up_flag = True
        self.spreader_lift_height = 0

        self.start_time = 0
        self.spreader_height_count = 0  # 吊具高度越界计数判断，连续计数超过3次，吊具确认吊具高度越界
        self.no_trolley = 0  # 对找不到横梁情景计数用
        try:
            self.config = self.load_cfg()
        except Exception:
            self.log.error("加载配置文件信息失败！\n" + traceback.format_exc())
        self.msg_fmt = {
            "deviceID": self.config["deviceID"],
            "RSSIF_num": 0,
            "RSSIB_num": 0,
            "RSSI_HF": -99,
            "RSSI_HB": -99,
            "spreaderHF": -99,
            "spreaderHB": -99,
            "spreaderH": -99,
            "PLC_spreaderH": -99,
            "delta": 0,
            "trolleyLocation": -99,
            "PLC_trolleyL": -99,
            "isLoad": "未载箱",
            "isOverShell": "未跨贝",
            "moveDir": "",
            "spreaderState": 0,
            "slowTime": 9999,
            "speed": 0,
            "conH": 999,
            "everyH": "",
            "hasBanner": "无",
            "bannerDis1": 999,
            "bannerDis2": 999,
            "inDanger": "否",
            "thisH": 999,
            "spreaderSpeed": 0,
            "lowDis": 999
        }
        self.row_height = [9] * self.config["rowNumber"]  # 表示每一排的箱子高度数据，单位为米
        self.last_row_height = [-99] * self.config["rowNumber"]  # 记录上一次的排位的箱子高度数据，单位为米,默认箱高位最高
        # 重复引用上次箱高数据计数
        for i in range(self.config["rowNumber"]):
            setattr(self, "row%s_duplicate_reference" % i, 0)
        # 重复引用上次吊具高度数据计数
        self.spreader_duplicate_reference = 0
        try:
            self.initFlidar()
            self.lidarF_last_count = self.lidarF.getCount()  # 心跳计数，判断扫描仪是否掉线
            self.initBlidar()
            self.lidarB_last_count = self.lidarB.getCount()  # 心跳计数，判断扫描仪是否掉线
        except Exception:
            self.log.error("初始化防打保龄扫描仪故障！\n" + traceback.format_exc())
        try:
            if self.config['PLCSwitch']:
                self.initPLC()
        except Exception:
            self.log.error("PLC连接初始化失败！\n" + traceback.format_exc())

    def __del__(self):
        if self.lidarF:
            self.lidarF.joinThread()
            self.lidarF.disconnect()
        if self.lidarB:
            self.lidarB.joinThread()
            self.lidarB.disconnect()
        if self.plc:
            self.plc.joinThread()
            self.plc.plc_disconnect()

    def load_cfg(self):
        with open('spsconfig.json', 'r') as f:
            config = json.load(f)
        return config

    def initBlidar(self):
        """
        初始化司机室后的扫描仪
        :return: 
        """
        if self.config["lidarType"] == 1:
            # 飞思迈尔扫描仪
            self.lidarB = FM_TCP(self.config["BlidarIP"], self.config["BlidarPort"],
                                 self.config["BlidarCID"], self.config["BlidarName"])
            self.lidarB.setXOffset(self.config["BXoffset"] + self.config["overallXoffset"])
            self.lidarB.setYOffset(self.config["BYoffset"] + self.config["overallYoffset"])
            self.lidarB.setThetaOffset(self.config["BThetaoffset"] + self.config["overallThetaoffset"])
            self.lidarB.setXReverse(self.config["BXReverse"])
            self.lidarB.setYReverse()
            self.lidarB.connect()
            self.lidarB.sendStart()
        elif self.config["lidarType"] == 2:
            #  sick扫描仪
            self.lidarB = Sick511(self.config["BlidarIP"], self.config["BlidarPort"],
                                  self.config["BlidarCID"], self.config["BlidarName"])
            self.lidarB.setXOffset(self.config["BXoffset"] + self.config["overallXoffset"])
            self.lidarB.setYOffset(self.config["BYoffset"] + self.config["overallYoffset"])
            self.lidarB.setThetaOffset(self.config["BThetaoffset"] + self.config["overallThetaoffset"])
            self.lidarB.setXReverse(self.config["BXReverse"])
            self.lidarB.setYReverse()
            self.lidarB.connect()
            self.lidarB.checkHardware()

    def initFlidar(self):
        """
        初始化司机室前的扫描仪参数
        :return: 
        """
        if self.config["lidarType"] == 1:
            self.lidarF = FM_TCP(self.config["FlidarIP"], self.config["FlidarPort"],
                                 self.config["FlidarCID"], self.config["FlidarName"])
            self.lidarF.setXOffset(self.config["FXoffset"] + self.config["overallXoffset"])
            self.lidarF.setYOffset(self.config["FThetaoffset"] + self.config["overallYoffset"])
            self.lidarF.setThetaOffset(self.config["FThetaoffset"] + self.config["overallThetaoffset"])
            self.lidarF.setXReverse(self.config["FXReverse"])
            self.lidarF.setYReverse()

            self.lidarF.connect()
            self.lidarF.sendStart()
        elif self.config["lidarType"] == 2:
            self.lidarF = Sick511(self.config["FlidarIP"], self.config["FlidarPort"],
                                  self.config["FlidarCID"], self.config["FlidarName"])
            self.lidarF.setXOffset(self.config["FXoffset"] + self.config["overallXoffset"])
            self.lidarF.setYOffset(self.config["FThetaoffset"] + self.config["overallYoffset"])
            self.lidarF.setThetaOffset(self.config["FThetaoffset"] + self.config["overallThetaoffset"])
            self.lidarF.setXReverse(self.config["FXReverse"])
            self.lidarF.setYReverse()

            self.lidarF.connect()
            self.lidarF.checkHardware()

    def initPLC(self):
        self.plc = PLC_TCP(self.config["PLCIP"], self.config["PLCPort"], self.config["PLCID"])
        self.plc.plc_connect()

    def format_send_msg(self):
        send_msg = b'\xaa'
        self.send_count += 1
        if self.send_count > 200:
            self.send_count = 0
        send_msg += pack(">B", self.send_count)
        # 司机室后方扫描仪状态
        if self.is_lidarB_dirty:
            send_msg += b'\x02'
        else:
            send_msg += b'\x01'
        # 司机室前方扫描仪状态
        if self.is_lidarF_dirty:
            send_msg += b'\x02'
        else:
            send_msg += b'\x02'
        # 司机室后方扫描仪数据是否正常
        if self.lidarB_count > self.config["aliveCount"]:
            send_msg += b'\x02'
        else:
            send_msg += b'\x01'
        # 司机室前方扫描仪数据是否正常
        if self.lidarF_count > 5:
            send_msg += b'\x02'
        else:
            send_msg += b'\x01'
        # 本呗集装箱的最高高度，单位为mm，2个字节
        self.real_row_height_send = [int(ele * 1000) for ele in self.row_height]  # 转换单位，并取整
        if len(self.real_row_height_send) != len(self.config["rowData"]):
            # 补全所有排位高度
            for i in range(len(self.config["rowData"]) - len(self.real_row_height_send)):
                self.real_row_height_send.append(0)
        send_msg += pack('>H', max(self.real_row_height_send))
        # 吊具高度，单位为mm，2个字节
        send_msg += pack(">H", int(self.spreader_height * 1000))
        # 小车当前的位置
        send_msg += pack(">L", int(self.trolley_location * 1000))
        # 17排的箱高数据
        send_msg += b''.join([pack(">H", ele) for ele in self.real_row_height_send])
        # 吊具下降速度控制 0~100%
        if self.spreader_warning:
            send_msg += b'\x00\x0a'
        else:
            send_msg += pack('>H', 100)

        # 小车向前运行保护
        send_msg += pack('>B', self.warning_flag_F + 1)
        # 小车向后运行保护
        send_msg += pack('>B', self.warning_flag_B + 1)
        # 吊具起升控制
        send_msg += self.chassis_lift_risk
        # 备用1字节
        send_msg += b'\x00'
        # 吊具前方与障碍物的距离
        send_msg += pack('>L', int(self.head_barrier * 1000))
        # 吊具后方与障碍物的距离
        send_msg += pack('>L', int(self.back_barrier * 1000))

        # 每一排的箱中心距离小车的距离值，单位为mm,保留符号，PLC作为方向判断
        send_msg += b''.join(
            [pack('>l', int((ele - self.trolley_location) * 1000)) for ele in self.config['rowData']])
        # 结束符
        send_msg += b'\xbb'
        return send_msg

    def plc_send(self, send_msg):
        self.plc.plc_send(send_msg)

    def plc_recv(self):
        temp = self.plc.get_recv()
        if temp:
            [self.trolley_location_plc, self.plc_spreader_height, self.is_spreader_load,
             self.is_rmg_moving, self.trolley_direction, self.is_spreader_down,
             self.current_speed, self.rated_time, self.rated_speed, self.container_size
             ] = temp
            self.msg_fmt["PLC_trolleyL"] = self.trolley_location_plc
            self.msg_fmt["PLC_spreaderH"] = self.plc_spreader_height
            self.msg_fmt["isLoad"] = "载箱" if self.is_spreader_load else "未载箱"
            self.msg_fmt["isOverShell"] = "跨贝" if self.is_rmg_moving else "未跨贝"
            self.msg_fmt["moveDir"] = "司机室前" if self.trolley_direction == 1 else "司机室后" \
                if self.trolley_direction == 2 else "停止"
            self.msg_fmt["spreaderState"] = "下降" if self.is_spreader_down == 2 else "上升" \
                if self.is_spreader_down == 1 else "停止"
            self.msg_fmt["slowTime"] = self.rated_time
            self.msg_fmt["speed"] = self.current_speed
            self.msg_fmt["conH"] = 2.596 if self.container_size == 1 else 2.896
            self.msg_fmt["delta"] = abs(self.plc_spreader_height - self.spreader_height)
            self.plc_msg_count = 0
        else:
            print('没有收到PLC的数据点！！！！')
            self.plc_msg_count += 1

    def plot(self):
        if self.config["plotSwitch"]:
            plt.clf()
            plt.ylim(-1, 25)  # 设置y轴的坐标范围,避免图像跳动影响观感
            # plt.axis([-60,20,-1,20])
            plt.title(self.config["deviceID"] + " 小车位置{} 吊具高度{}".format(
                round(self.trolley_location, 2), round(self.spreader_height, 2)
            ), color='purple', fontproperties='SimHei')
            plt.xlim(self.left_limit, self.right_limit)
            # plt.plot(self.lidarB_x, self.lidarB_y, 'g.')
            # plt.plot(self.lidarF_x, self.lidarF_y, 'b.')
            plt.plot(self.lidar_all_x, self.lidar_all_y, 'b.')
            # plt.plot(0, self.second_spreader_height, 'r*')
            plt.plot(0, self.spreader_height, 'g*')
            # plt.plot(self.rssi_temp_x, self.rssi_temp_y, "r*")
            # plt.plot([-50,60],[6.2, 6.2],"-")
            # plt.plot([0,0], [0,24], "-")

            # 显示每一个排位的高度
            if self.config["beamPosition"] < 0:
                row_x = [self.config["rowData"][i] - self.trolley_location for i in range(self.config["rowNumber"])]
            else:
                row_x = [self.trolley_location - self.config["rowData"][i] for i in range(self.config["rowNumber"])]
            plt.plot(row_x, self.row_height, "r*")

            plt.plot([0, 0], [-0.3, 24], '-')  # 显示中心线
            # 和当前吊具高位线
            plt.plot([self.left_limit, self.right_limit], [self.spreader_height, self.spreader_height], '-')
            plt.draw()
            plt.pause(0.001)
        else:
            plt.close()
            sleep(0.1)

    def is_lidar_alive(self):
        # 扫描仪的有效性判断，对不正常的扫描仪进行计数判断
        lidarB_count = self.lidarB.getCount()
        lidarF_count = self.lidarF.getCount()

        if lidarB_count == self.lidarB_last_count:
            self.lidarB_count += 1
        else:
            self.lidarB_last_count = lidarB_count
            self.lidarB_count = 0

        if lidarF_count == self.lidarF_last_count:
            self.lidarF_count += 1
        else:
            self.lidarF_last_count = lidarF_count
            self.lidarF_count = 0

        if self.lidarB_count > self.config["aliveCount"]:
            self.log.warning("司机室后扫描仪心跳不正常,重新连接")
            self.lidarB_state = False

        if self.lidarF_count > self.config["aliveCount"]:
            self.log.warning("司机室前扫描仪心跳不正常，重新连接")
            self.lidarF_state = False

    def dataLog(self, x, y):
        if self.config['dataLog']:
            now = datetime.now().strftime('-%m-%d')
            fileName = self.logPrefix + now + '.txt'
            timeStamp = strftime("%Y-%m-%d %H:%M:%S", localtime())
            with open(fileName, 'a') as file:
                file.write(timeStamp + '\n' + str(x) + '\n' + str(y) + '\n')

    def cal_spreaderF(
            self,
            minSpreaderLevel,
            maxSpreaderLevel,
            minSpreaderHeight,
            maxSpreaderHeight,
            minRSSIValue,
            distanceRSSI):
        # 用于计算吊具高度的司机室前扫描仪得到的高反射率点坐标容器
        RSSIF_y = []
        # 高亮点计算失败时进行普通点运算的冗余容器
        tempB_y = []
        tempF_y = []
        # 司机室前侧扫描仪计算的吊具高度记录
        first_spreader_height = 0
        # 司机室后侧扫描仪计算吊具高度记录
        second_spreader_height = 0
        RSSIF = self.lidarF.getRSSI()
        if len(self.lidarF_x) == len(RSSIF):
            for i in range(len(self.lidarF_x)):
                if RSSIF[i] > minRSSIValue and minSpreaderHeight < self.lidarF_y[i] < maxSpreaderHeight and \
                        minSpreaderLevel < self.lidarF_x[i] < maxSpreaderLevel:
                    RSSIF_y.append(self.lidarF_y[i])
            for i in range(len(self.lidarB_x)):
                if minSpreaderLevel < self.lidarB_x[i] < maxSpreaderLevel and \
                        minSpreaderHeight < self.lidarB_y[i] < maxSpreaderHeight:
                    tempB_y.append(self.lidarB_y[i])
        else:
            # 长度校验错误，则发送一次心跳包
            print('高亮点长度校验错误：{} ???????????????????????????????????'.format(len(RSSIF)))
            # 通过更改scan_counter值间接控制发送心跳包
            self.scan_counter = self.config['scanConfigRotation'] + 1
            for i in range(len(self.lidarF_x)):
                if minSpreaderLevel < self.lidarF_x[i] < maxSpreaderLevel and \
                        minSpreaderHeight < self.lidarF_y[i] < maxSpreaderHeight:
                    tempF_y.append(self.lidarF_y[i])
            for i in range(len(self.lidarB_x)):
                if minSpreaderLevel < self.lidarB_x[i] < maxSpreaderLevel and \
                        minSpreaderHeight < self.lidarB_y[i] < maxSpreaderHeight:
                    tempB_y.append(self.lidarB_y[i])
        self.msg_fmt["RSSIF_num"] = len(RSSIF_y)
        if RSSIF_y:
            # 根据高亮贴返回点数据计算吊具高度
            self.spreader_height = np.median(RSSIF_y) - distanceRSSI  # 吊具高亮版距离吊具底端的距离
            self.msg_fmt["RSSI_HF"] = round(self.spreader_height, 3)
            self.msg_fmt["spreaderHF"] = -99
        else:
            # 根据普通点数据计算冗余高度
            if tempF_y:
                first_spreader_height = np.median(sorted(tempF_y)[-int(len(tempF_y) * 0.3):]) - distanceRSSI
                self.msg_fmt["RSSI_HF"] = -99
                self.msg_fmt["spreaderHF"] = round(first_spreader_height, 3)
            else:
                # 根据前侧扫描仪数据计算吊具高度失败
                first_spreader_height = -99
                self.msg_fmt["RSSI_HF"] = -99
                self.msg_fmt["spreaderHF"] = -99
            if tempB_y:
                # 高亮点计算失败
                # 计算后侧扫描仪所得数据
                second_spreader_height = np.median(sorted(tempB_y)[-int(len(tempB_y) * 0.3):]) - distanceRSSI
                self.msg_fmt["RSSI_HB"] = -99
                self.msg_fmt["spreaderHB"] = round(second_spreader_height, 3)
            else:
                # 后侧扫描仪数据计算吊具高度失败
                second_spreader_height = -99
                self.msg_fmt["RSSI_HB"] = -99
                self.msg_fmt["spreaderHB"] = -99

            if second_spreader_height == -99 and first_spreader_height == -99 and \
                    self.spreader_duplicate_reference < 3:
                self.log.warning('吊具高度计算失败！启用上次记录值, 上次吊具高度数值为：%s' % self.last_spreader_height)
                self.spreader_height = self.last_spreader_height
            elif second_spreader_height == -99 and first_spreader_height == -99 and \
                    self.spreader_duplicate_reference >= 3:
                # 吊具高度采用上次记录值超过三次，不再继续采用上次记录值，采用实时数据
                self.log.warning("多次采用上次记录值，此后采用默认吊具高度数据：-99")
                self.spreader_height = -99
                # 记录异常错误点云数据
                self.dataLog(self.lidarF_x + self.lidarB_x, self.lidarF_y + self.lidarB_y)
            elif first_spreader_height == -99 and second_spreader_height != -99:
                self.spreader_height = second_spreader_height
                self.spreader_duplicate_reference = 0
            elif second_spreader_height == -99 and first_spreader_height != -99:
                self.spreader_height = first_spreader_height
                self.spreader_duplicate_reference = 0
            else:
                # 取两个吊具高度中相对上次高度数据变化较小者作为参考
                if abs(second_spreader_height - self.last_spreader_height) > \
                        abs(first_spreader_height - self.last_spreader_height):
                    self.spreader_height = first_spreader_height
                else:
                    self.spreader_height = second_spreader_height
                self.spreader_duplicate_reference = 0

        # 对吊具高度进行判断是否在正常的范围 不超过16.7米
        if self.spreader_height > 21:
            self.spreader_height_count += 1
            # 连续越界，则确认吊具高度越界。否则视为干扰点处理，使用上次值
            if self.spreader_height_count > 3:
                self.spreader_height = self.last_spreader_height
                self.log.warning('吊具高度异常{}，上次记录值{} 前侧扫描仪计算吊具高度{} 后侧扫描仪计算吊具高度{},异常计数{}，小车位置{}'.format(
                    self.spreader_height, self.last_spreader_height, first_spreader_height,
                    second_spreader_height, self.spreader_height_count, self.trolley_location))
                self.dataLog(self.lidarF_x + self.lidarB_x, self.lidarF_y + self.lidarB_y)
            else:
                self.spreader_height = self.last_spreader_height

        else:
            self.spreader_height_count = 0

        if abs(self.spreader_height - self.last_spreader_height) > 2 and self.last_spreader_height != 0:
            # 此次计算的吊具高度相比上次跳变巨大，超过2米,不可靠
            time_gap = time() - self.start_time
            # 时间间隔很短时，则认为高度突时由吊具摇动剧烈造成
            if time_gap < 0.5:
                if self.spreader_error_count < 5:
                    self.spreader_error_count += 1
                    self.spreader_height = self.last_spreader_height
            else:
                self.log.warning('时间间隔{}，吊具高度{}突变，上次记录值{}, 前侧扫描仪计算吊具高度{} 高亮点吊具高度{}，小车位置{}'.format(
                    round(time_gap, 3), self.spreader_height, self.last_spreader_height,
                    first_spreader_height, second_spreader_height, self.trolley_location))
                self.dataLog(self.lidarF_x + self.lidarB_x, self.lidarF_y + self.lidarB_y)

        if self.spreader_height < 0:
            self.spreader_height = 0

        self.last_spreader_height = self.spreader_height
        self.msg_fmt["spreaderH"] = round(self.spreader_height, 3)

    def cal_spreaderBoth(
            self,
            minSpreaderLevel,
            maxSpreaderLevel,
            minSpreaderHeight,
            maxSpreaderHeight,
            minRSSIValue,
            distanceRSSI):
        # 用于计算吊具高度的司机室前扫描仪得到的高反射率点坐标容器
        RSSIF_y = []
        RSSIB_y = []
        # 使用高亮点计算吊具高度是否成功
        RSSIF_done = False
        RSSIB_done = False
        # 高亮点计算失败时进行普通点运算的冗余容器
        tempB_y = []
        tempF_y = []
        # 司机室前侧扫描仪计算的吊具高度记录
        first_spreader_height = 0
        # 司机室后侧扫描仪计算吊具高度记录
        second_spreader_height = 0
        RSSIF = self.lidarF.getRSSI()
        RSSIB = self.lidarB.getRSSI()
        if len(self.lidarF_x) == len(RSSIF):
            for i in range(len(self.lidarF_x)):
                if RSSIF[i] > minRSSIValue and minSpreaderHeight < self.lidarF_y[i] < maxSpreaderHeight and \
                        minSpreaderLevel < self.lidarF_x[i] < maxSpreaderLevel:
                    RSSIF_y.append(self.lidarF_y[i])
        else:
            # 长度校验错误，则发送一次心跳包
            print('前侧扫描仪高亮点长度校验错误：{} ???????????????????????????????????'.format(len(RSSIF)))
            # 通过更改scan_counter值间接控制发送心跳包
            self.scan_counter = self.config['scanConfigRotation'] + 1
            for i in range(len(self.lidarF_x)):
                if minSpreaderLevel < self.lidarF_x[i] < maxSpreaderLevel and \
                        minSpreaderHeight < self.lidarF_y[i] < maxSpreaderHeight:
                    tempF_y.append(self.lidarF_y[i])

        if len(self.lidarB_x) == len(RSSIB):
            for i in range(len(self.lidarB_x)):
                if RSSIB[i] > minRSSIValue and minSpreaderHeight < self.lidarB_y[i] < maxSpreaderHeight and \
                        minSpreaderLevel < self.lidarB_x[i] < maxSpreaderLevel:
                    RSSIB_y.append(self.lidarB_y[i])
        else:
            # 长度校验错误，则发送一次心跳包
            print('后侧扫描仪高亮点长度校验错误：{} ???????????????????????????????????'.format(len(RSSIB)))
            # 通过更改scan_counter值间接控制发送心跳包
            self.scan_counter = self.config['scanConfigRotation'] + 1
            for i in range(len(self.lidarB_x)):
                if minSpreaderLevel < self.lidarB_x[i] < maxSpreaderLevel and \
                        minSpreaderHeight < self.lidarB_y[i] < maxSpreaderHeight:
                    tempB_y.append(self.lidarB_y[i])

        self.msg_fmt["RSSIF_num"] = len(RSSIF_y)
        self.msg_fmt["RSSIB_num"] = len(RSSIB_y)
        if RSSIF_y:
            # 根据高亮贴返回点数据计算吊具高度
            first_spreader_height = np.median(RSSIF_y) - distanceRSSI  # 吊具高亮版距离吊具底端的距离
            self.msg_fmt["RSSI_HF"] = round(first_spreader_height, 3)
            self.msg_fmt["spreaderHF"] = -99
            RSSIF_done = True

        if RSSIB_y:
            # 根据高亮贴返回点数据计算吊具高度
            second_spreader_height = np.median(RSSIB_y) - distanceRSSI  # 吊具高亮版距离吊具底端的距离
            self.msg_fmt["RSSI_HB"] = round(second_spreader_height, 3)
            self.msg_fmt["spreaderHB"] = -99
            RSSIB_done = True

        if (not RSSIF_done) and (not RSSIB_done):
            # 两侧的高亮点计算均失败
            # 根据普通点数据计算冗余高度
            if tempF_y:
                first_spreader_height = np.median(sorted(tempF_y)[-int(len(tempF_y) * 0.3):]) - distanceRSSI
                self.msg_fmt["RSSI_HF"] = -99
                self.msg_fmt["spreaderHF"] = round(first_spreader_height, 3)
            else:
                # 根据前侧扫描仪数据计算吊具高度失败
                first_spreader_height = -99
                self.msg_fmt["RSSI_HF"] = -99
                self.msg_fmt["spreaderHF"] = -99
            # 根据普通点数据计算冗余高度
            if tempB_y:
                second_spreader_height = np.median(sorted(tempB_y)[-int(len(tempB_y) * 0.3):]) - distanceRSSI
                self.msg_fmt["RSSI_HB"] = -99
                self.msg_fmt["spreaderHB"] = round(second_spreader_height, 3)
            else:
                # 根据前侧扫描仪数据计算吊具高度失败
                second_spreader_height = -99
                self.msg_fmt["RSSI_HB"] = -99
                self.msg_fmt["spreaderHB"] = -99

            if second_spreader_height == -99 and first_spreader_height == -99 and \
                    self.spreader_duplicate_reference < 3:
                self.log.warning('吊具高度计算失败！启用上次记录值, 上次吊具高度数值为：%s' % self.last_spreader_height)
                self.spreader_height = self.last_spreader_height
            elif second_spreader_height == -99 and first_spreader_height == -99 and \
                    self.spreader_duplicate_reference >= 3:
                # 吊具高度采用上次记录值超过三次，不再继续采用上次记录值，采用实时数据
                self.log.warning("多次采用上次记录值，此后采用默认吊具高度数据：-99")
                self.spreader_height = -99
                # 记录异常错误点云数据
                self.dataLog(self.lidarF_x + self.lidarB_x, self.lidarF_y + self.lidarB_y)
            elif first_spreader_height == -99 and second_spreader_height != -99:
                self.spreader_height = second_spreader_height
                self.spreader_duplicate_reference = 0
            elif second_spreader_height == -99 and first_spreader_height != -99:
                self.spreader_height = first_spreader_height
                self.spreader_duplicate_reference = 0
            else:
                # 取两个吊具高度中相对上次高度数据变化较小者作为参考
                if abs(second_spreader_height - self.last_spreader_height) > \
                        abs(first_spreader_height - self.last_spreader_height):
                    self.spreader_height = first_spreader_height
                else:
                    self.spreader_height = second_spreader_height
                self.spreader_duplicate_reference = 0
        elif (not RSSIF_done) and RSSIB_done:
            # 仅后侧扫描仪高亮点计算成功，优先使用
            self.msg_fmt["RSSI_HF"] = -99
            self.spreader_height = second_spreader_height
        elif RSSIF_done and (not RSSIB_done):
            # 仅前侧扫描仪高亮点计算成功，优先使用
            self.msg_fmt["RSSI_HB"] = -99
            self.spreader_height = first_spreader_height

        # 对吊具高度进行判断是否在正常的范围 不超过16.7米
        if self.spreader_height > 21:
            self.spreader_height_count += 1
            # 连续越界，则确认吊具高度越界。否则视为干扰点处理，使用上次值
            if self.spreader_height_count > 3:
                self.spreader_height = self.last_spreader_height
                self.log.warning('吊具高度异常{}，上次记录值{} 前侧扫描仪计算吊具高度{} 后侧扫描仪计算吊具高度{},异常计数{}，小车位置{}'.format(
                    self.spreader_height, self.last_spreader_height, first_spreader_height,
                    second_spreader_height, self.spreader_height_count, self.trolley_location))
                self.dataLog(self.lidarF_x + self.lidarB_x, self.lidarF_y + self.lidarB_y)
            else:
                self.spreader_height = self.last_spreader_height

        else:
            self.spreader_height_count = 0

        if abs(self.spreader_height - self.last_spreader_height) > 2 and self.last_spreader_height != 0:
            # 此次计算的吊具高度相比上次跳变巨大，超过2米,不可靠
            time_gap = time() - self.start_time
            # 时间间隔很短时，则认为高度突时由吊具摇动剧烈造成
            if time_gap < 0.5:
                if self.spreader_error_count < 5:
                    self.spreader_error_count += 1
                    self.spreader_height = self.last_spreader_height
            else:
                self.log.warning('时间间隔{}，吊具高度{}突变，上次记录值{}, 前侧扫描仪计算吊具高度{} 高亮点吊具高度{}，小车位置{}'.format(
                    round(time_gap, 3), self.spreader_height, self.last_spreader_height,
                    first_spreader_height, second_spreader_height, self.trolley_location))
                self.dataLog(self.lidarF_x + self.lidarB_x, self.lidarF_y + self.lidarB_y)

        if self.spreader_height < 0:
            self.spreader_height = 0

        self.last_spreader_height = self.spreader_height
        self.msg_fmt["spreaderH"] = round(self.spreader_height, 3)

    def cal_spreaderB(
            self,
            minSpreaderLevel,
            maxSpreaderLevel,
            minSpreaderHeight,
            maxSpreaderHeight,
            minRSSIValue,
            distanceRSSI):
        # 用于计算吊具高度的司机室前扫描仪得到的高反射率点坐标容器
        RSSIB_y = []
        # 高亮点计算失败时进行普通点运算的冗余容器
        tempB_y = []
        tempF_y = []
        # 司机室前侧扫描仪计算的吊具高度记录
        first_spreader_height = 0
        # 司机室后侧扫描仪计算吊具高度记录
        second_spreader_height = 0
        RSSIB = self.lidarB.getRSSI()
        if len(self.lidarB_x) == len(RSSIB):
            for i in range(len(self.lidarB_x)):
                if RSSIB[i] > minRSSIValue and minSpreaderHeight < self.lidarB_y[i] < maxSpreaderHeight and \
                        minSpreaderLevel < self.lidarB_x[i] < maxSpreaderLevel:
                    RSSIB_y.append(self.lidarB_y[i])
            for i in range(len(self.lidarF_x)):
                if minSpreaderLevel < self.lidarF_x[i] < maxSpreaderLevel and \
                        minSpreaderHeight < self.lidarF_y[i] < maxSpreaderHeight:
                    tempF_y.append(self.lidarF_y[i])
        else:
            # 长度校验错误，则发送一次心跳包
            print('高亮点长度校验错误：{} ???????????????????????????????????'.format(len(RSSIB)))
            # 通过更改scan_counter值间接控制发送心跳包
            self.scan_counter = self.config['scanConfigRotation'] + 1
            for i in range(len(self.lidarF_x)):
                if minSpreaderLevel < self.lidarF_x[i] < maxSpreaderLevel and \
                        minSpreaderHeight < self.lidarF_y[i] < maxSpreaderHeight:
                    tempF_y.append(self.lidarF_y[i])
            for i in range(len(self.lidarB_x)):
                if minSpreaderLevel < self.lidarB_x[i] < maxSpreaderLevel and \
                        minSpreaderHeight < self.lidarB_y[i] < maxSpreaderHeight:
                    tempB_y.append(self.lidarB_y[i])
        self.msg_fmt["RSSIB_num"] = len(RSSIB_y)
        if RSSIB_y:
            # 根据高亮贴返回点数据计算吊具高度
            self.spreader_height = np.median(RSSIB_y) - distanceRSSI  # 吊具高亮版距离吊具底端的距离
            self.msg_fmt["RSSI_HB"] = round(self.spreader_height, 3)
            self.msg_fmt["spreaderHB"] = -99
        else:
            # 根据普通点数据计算冗余高度
            if tempB_y:
                first_spreader_height = np.median(sorted(tempB_y)[-int(len(tempB_y) * 0.3):]) - distanceRSSI
                self.msg_fmt["RSSI_HB"] = -99
                self.msg_fmt["spreaderHB"] = round(first_spreader_height, 3)
            else:
                # 根据前侧扫描仪数据计算吊具高度失败
                first_spreader_height = -99
                self.msg_fmt["RSSI_HB"] = -99
                self.msg_fmt["spreaderHB"] = -99

            if tempF_y:
                # 计算前侧扫描仪所得数据
                second_spreader_height = np.median(sorted(tempF_y)[-int(len(tempF_y) * 0.3):]) - distanceRSSI
                self.msg_fmt["RSSI_HF"] = -99
                self.msg_fmt["spreaderHF"] = round(second_spreader_height, 3)
            else:
                # 后侧扫描仪数据计算吊具高度失败
                second_spreader_height = -99
                self.msg_fmt["RSSI_HF"] = -99
                self.msg_fmt["spreaderHF"] = -99

            if second_spreader_height == -99 and first_spreader_height == -99 and \
                    self.spreader_duplicate_reference < 3:
                self.log.warning('吊具高度计算失败！启用上次记录值, 上次吊具高度数值为：%s' % self.last_spreader_height)
                self.spreader_height = self.last_spreader_height
            elif second_spreader_height == -99 and first_spreader_height == -99 and \
                    self.spreader_duplicate_reference >= 3:
                # 吊具高度采用上次记录值超过三次，不再继续采用上次记录值，采用实时数据
                self.log.warning("多次采用上次记录值，此后采用默认吊具高度数据：-99")
                self.spreader_height = -99
                # 记录异常错误点云数据
                self.dataLog(self.lidarF_x + self.lidarB_x, self.lidarF_y + self.lidarB_y)
            elif first_spreader_height == -99 and second_spreader_height != -99:
                self.spreader_height = second_spreader_height
                self.spreader_duplicate_reference = 0
            elif second_spreader_height == -99 and first_spreader_height != -99:
                self.spreader_height = first_spreader_height
                self.spreader_duplicate_reference = 0
            else:
                # 取两个吊具高度中相对上次高度数据变化较小者作为参考
                if abs(second_spreader_height - self.last_spreader_height) > \
                        abs(first_spreader_height - self.last_spreader_height):
                    self.spreader_height = first_spreader_height
                else:
                    self.spreader_height = second_spreader_height
                self.spreader_duplicate_reference = 0

        # 对吊具高度进行判断是否在正常的范围 不超过16.7米
        if self.spreader_height > 21:
            self.spreader_height_count += 1
            # 连续越界，则确认吊具高度越界。否则视为干扰点处理，使用上次值
            if self.spreader_height_count > 3:
                self.spreader_height = self.last_spreader_height
                self.log.warning('吊具高度异常{}，上次记录值{} 前侧扫描仪计算吊具高度{} 后侧扫描仪计算吊具高度{},异常计数{}，小车位置{}'.format(
                    self.spreader_height, self.last_spreader_height, first_spreader_height,
                    second_spreader_height, self.spreader_height_count, self.trolley_location))
                self.dataLog(self.lidarF_x + self.lidarB_x, self.lidarF_y + self.lidarB_y)
            else:
                self.spreader_height = self.last_spreader_height

        else:
            self.spreader_height_count = 0

        if abs(self.spreader_height - self.last_spreader_height) > 2 and self.last_spreader_height != 0:
            # 此次计算的吊具高度相比上次跳变巨大，超过2米,不可靠
            time_gap = time() - self.start_time
            # 时间间隔很短时，则认为高度突时由吊具摇动剧烈造成
            if time_gap < 0.5:
                if self.spreader_error_count < 5:
                    self.spreader_error_count += 1
                    self.spreader_height = self.last_spreader_height
            else:
                self.log.warning('时间间隔{}，吊具高度{}突变，上次记录值{}, 前侧扫描仪计算吊具高度{} 高亮点吊具高度{}，小车位置{}'.format(
                    round(time_gap, 3), self.spreader_height, self.last_spreader_height,
                    first_spreader_height, second_spreader_height, self.trolley_location))
                self.dataLog(self.lidarF_x + self.lidarB_x, self.lidarF_y + self.lidarB_y)

        if self.spreader_height < 0:
            self.spreader_height = 0

        self.last_spreader_height = self.spreader_height
        self.msg_fmt["spreaderH"] = round(self.spreader_height, 3)

    def cal_spreader_height(self):
        if self.config["RSSIChoice"] < 0:  # -1
            #  使用司机室后侧的扫描仪扫描高亮贴
            self.cal_spreaderB(
                self.config["minSpreaderLevel"],
                self.config["maxSpreaderLevel"],
                self.config["minSpreaderHeight"],
                self.config["maxSpreaderHeight"],
                self.config["minRSSIValue"],
                self.config["distanceRSSI"])
        elif 0 < self.config["RSSIChoice"] < 2:  # 1
            # 使用司机室前的扫描仪扫描高亮贴
            self.cal_spreaderF(
                self.config["minSpreaderLevel"],
                self.config["maxSpreaderLevel"],
                self.config["minSpreaderHeight"],
                self.config["maxSpreaderHeight"],
                self.config["minRSSIValue"],
                self.config["distanceRSSI"])
        else:  # 2
            # 双侧扫描仪均扫描高亮贴
            self.cal_spreaderBoth(
                self.config["minSpreaderLevel"],
                self.config["maxSpreaderLevel"],
                self.config["minSpreaderHeight"],
                self.config["maxSpreaderHeight"],
                self.config["minRSSIValue"],
                self.config["distanceRSSI"])

    def cal_trolley_location(
            self,
            beamPosition,
            minGantryHeight,
            maxGantryHeight):
        # 计算小车位置
        trolley = []
        if beamPosition < 0:
            if self.config["beamLidar"] > 0:
                for i in range(len(self.lidarF_y)):
                    if minGantryHeight < self.lidarF_y[i] < maxGantryHeight and \
                            self.lidarF_x[i] < -abs(self.config["FXoffset"]) - 1:
                        trolley.append(self.lidarF_x[i])
                if not trolley:
                    if self.no_trolley < 10:
                        self.no_trolley += 1
                        print('当前没有找到横梁???????????????????????????????????????')
                        # self.internal_error = 4
                        # 找不到端梁值时，使用上一次的值
                        self.trolley_location = self.last_trolley_location
                        self.log.warning('当前没有找到横梁值，启用上一次的值:{}'.format(self.trolley_location))
                        self.dataLog(self.lidarF_x, self.lidarF_y)
                else:
                    self.no_trolley = 0
                    self.trolley_location = abs(np.median(trolley))
                    self.last_trolley_location = self.trolley_location
                if self.no_trolley >= 10:
                    # 10次循环后仍然找不到横梁，采用PLC反馈的小车位置数据，并补偿偏移值
                    self.log.warning("多次未获取到横梁值，采用PLC小车位置数据运算。")
                    self.trolley_location = self.trolley_location_plc + self.config["trolley_plc_offset"]
                    self.dataLog(self.lidarF_x, self.lidarF_y)
            else:
                for i in range(len(self.lidarB_y)):
                    if minGantryHeight < self.lidarB_y[i] < maxGantryHeight and \
                            self.lidarB_x[i] < -abs(self.config["FXoffset"]) - 1:
                        trolley.append(self.lidarB_x[i])
                if not trolley:
                    if self.no_trolley < 10:
                        self.no_trolley += 1
                        print('当前没有找到横梁???????????????????????????????????????')
                        # self.internal_error = 4
                        # 找不到端梁值时，使用上一次的值
                        self.trolley_location = self.last_trolley_location
                        self.log.warning('当前没有找到横梁值，启用上一次的值:{}'.format(self.trolley_location))
                        self.dataLog(self.lidarB_x, self.lidarB_y)
                else:
                    self.no_trolley = 0
                    self.trolley_location = abs(np.median(trolley))
                    self.last_trolley_location = self.trolley_location
                if self.no_trolley >= 10:
                    # 10次循环后仍然找不到横梁，采用PLC反馈的小车位置数据，并补偿偏移值
                    self.log.warning("多次未获取到横梁值，采用PLC小车位置数据运算。")
                    self.trolley_location = self.trolley_location_plc + self.config["trolley_plc_offset"]
                    self.dataLog(self.lidarB_x, self.lidarB_y)
        else:
            if self.config["beamLidar"] < 0:
                for i in range(len(self.lidarB_y)):
                    if minGantryHeight < self.lidarB_y[i] < maxGantryHeight and \
                            self.lidarB_x[i] > abs(self.config["BXoffset"]) + 1:
                        trolley.append(self.lidarB_x[i])
                if not trolley:
                    if self.no_trolley < 10:
                        self.no_trolley += 1
                        print('当前没有找到横梁???????????????????????????????????????')
                        # 找不到端梁值时，使用上一次的值
                        self.trolley_location = self.last_trolley_location
                        self.log.warning('当前没有找到横梁值，启用上一次的值:{}'.format(self.trolley_location))
                        self.dataLog(self.lidarB_x, self.lidarB_y)
                else:
                    self.no_trolley = 0
                    self.trolley_location = abs(np.median(trolley))
                    self.last_trolley_location = self.trolley_location
                if self.no_trolley >= 10:
                    # 10次循环后仍然找不到横梁，采用PLC反馈的小车位置数据，并补偿偏移值
                    self.log.warning("多次未获取到横梁值，采用PLC小车位置数据运算。")
                    self.trolley_location = self.trolley_location_plc + self.config["trolley_plc_offset"]
                    self.dataLog(self.lidarF_x, self.lidarF_y)
            else:
                for i in range(len(self.lidarF_y)):
                    if minGantryHeight < self.lidarF_y[i] < maxGantryHeight and \
                            self.lidarF_x[i] > abs(self.config["BXoffset"]) + 1:
                        trolley.append(self.lidarF_x[i])
                if not trolley:
                    if self.no_trolley < 10:
                        self.no_trolley += 1
                        print('当前没有找到横梁???????????????????????????????????????')
                        # 找不到端梁值时，使用上一次的值
                        self.trolley_location = self.last_trolley_location
                        self.log.warning('当前没有找到横梁值，启用上一次的值:{}'.format(self.trolley_location))
                        self.dataLog(self.lidarF_x, self.lidarF_y)
                else:
                    self.no_trolley = 0
                    self.trolley_location = abs(np.median(trolley))
                    self.last_trolley_location = self.trolley_location
                if self.no_trolley >= 10:
                    # 10次循环后仍然找不到横梁，采用PLC反馈的小车位置数据，并补偿偏移值
                    self.log.warning("多次未获取到横梁值，采用PLC小车位置数据运算。")
                    self.trolley_location = self.trolley_location_plc + self.config["trolley_plc_offset"]
                    self.dataLog(self.lidarF_x, self.lidarF_y)
        self.msg_fmt["trolleyLocation"] = round(self.trolley_location, 3)

    def cut_interference(self):
        # 裁剪掉干扰点和无用点
        if self.config["beamPosition"] < 0:
            self.left_limit = - self.trolley_location - 0.1
            self.right_limit = self.config["girderLength"] - self.trolley_location + 0.1
        else:
            self.left_limit = self.trolley_location - self.config["girderLength"] - 0.1
            self.right_limit = self.trolley_location + 0.1
        # 切掉干扰点 切割不需要部分
        if self.is_spreader_load:
            cut_height = 3
        else:
            cut_height = 0.2
        i = 0
        while i < len(self.lidar_all_x):
            # 切掉高于横梁的部分
            if self.lidar_all_y[i] > self.config["maxGantryHeight"]:

                del self.lidar_all_x[i]
                del self.lidar_all_y[i]

            # 切掉小车底扫描仪附近干扰物
            elif self.config["trolleyLeftCut"] < self.lidar_all_x[i] < self.config["trolleyRightCut"] and \
                    self.lidar_all_y[i] > self.config["trolleyBottomCut"]:
                del self.lidar_all_x[i]
                del self.lidar_all_y[i]

            # 切掉大梁两端外侧
            elif self.lidar_all_x[i] < self.left_limit:
                del self.lidar_all_x[i]
                del self.lidar_all_y[i]
            elif self.lidar_all_x[i] > self.right_limit:
                del self.lidar_all_x[i]
                del self.lidar_all_y[i]

            # 切掉吊具中心左右各2米即切掉吊具，排除计算箱高时吊具的影响
            elif 2 > self.lidar_all_x[i] > -2 and self.lidar_all_y[i] > self.spreader_height - cut_height:
                del self.lidar_all_x[i]
                del self.lidar_all_y[i]
            else:
                i += 1

    def cal_row_height(self):
        # 计算每一排的箱高
        row = [[], [], [], [], [], [], [],
               [], [], [], [], [], [], [], [], [], []]
        # 计算点到横梁的距离
        if self.config["beamPosition"] < 0:
            # 取司机室前部端梁为参考
            yard_x = [ele + self.trolley_location for ele in self.lidar_all_x]
        else:
            # 取司机室后部端梁为参考
            yard_x = [self.trolley_location - ele for ele in self.lidar_all_x]
        for fm_index in range(len(yard_x)):
            for index in range(self.config["rowNumber"]):
                # 排位宽度为2.8米，只取中心的0.8米区作为计算域。
                if self.config["rowData"][index] - self.config["threshold"] < \
                        yard_x[fm_index] < self.config["rowData"][index] + self.config["threshold"]:
                    row[index].append(self.lidar_all_y[fm_index])

        # 记录小车运动中扫到的异常点
        moving_point_ex = []
        for index in range(self.config["rowNumber"]):
            if len(row[index]) > 2:
                # 使用中值计算箱高，确保计算出来的值是扫描到的真实箱高点。
                # self.row_height[index] = np.median(row[index])
                # 使用平均值计算箱高，可以滤掉由于吊箱光斑和堆场内光斑叠加造成的悬空点。
                # 使用平均值计算法箱高一定要确保箱子中心和实际箱子中心重合
                self.row_height[index] = np.median(row[index])
                # 扫描仪晃动导致的异常，记录下来索引进行后续处理
                if self.row_height[index] > 15:
                    moving_point_ex.append(index)
                # 正常得到数据，将重复引用上次箱高计数清零
                setattr(self, "row%s_duplicate_reference" % index, 0)
            else:
                if self.is_rmg_moving:  # 大车有跨呗操作
                    self.row_height[index] = -99  # 标记扫描不到的位置
                # 重复引用上次数据达小于3次则继续引用上次数据
                elif getattr(self, "row%s_duplicate_reference" % index) < 3:
                    self.row_height[index] = self.last_row_height[index]
                    setattr(self, "row%s_duplicate_reference" % index,
                            getattr(self, "row%s_duplicate_reference" % index) + 1)
                else:
                    # 重复引用上次数据达3次后就不在引用上次数据，防止大雾产生的异常数据干扰遗留
                    # 置为-99，则后续会取相邻最高箱高数据
                    self.row_height[index] = -99
        # 计算本贝最大箱高，排除由于扫描仪晃动扫到的异常高点
        cal_max = self.row_height.copy()
        for i in self.row_height:
            if i > 15:
                cal_max.remove(i)
        max_height = max(cal_max)
        cal_max.clear()
        # 将扫到的高于15米的异常高点设置为本贝最高箱点
        for i in moving_point_ex:
            self.row_height[i] = max_height
        for index in range(1, len(self.row_height) - 1):
            if self.row_height[index] == -99:
                # 如果某一排箱没有扫到点云，则取相邻的最高箱数据作为当前排的箱高，防止发生打保龄
                m = max(self.row_height[index - 1], self.row_height[index + 1])
                if m == -99:
                    # 如果相邻的排都没有点云数据，则取当前贝位最高箱数据作为本排的箱高
                    self.row_height[index] = max_height
                else:
                    self.row_height[index] = m
        if self.row_height[0] == -99:
            self.row_height[0] = self.row_height[1]
        if self.row_height[-1] == -99:
            self.row_height[-1] = self.row_height[-2]

        # 将低于0的值设置为0，不能有负数值
        self.row_height = [(0 if ele < 0 else ele) for ele in self.row_height]
        # 记录本次箱高数据，用作下次计算比较
        self.last_row_height = self.row_height
        self.msg_fmt["everyH"] = str([round(i, 3) for i in self.row_height])

    def is_trolley_safe(self):
        # 检测小车前后方向12米内是否有障碍物，进行防护保护
        head_area_x = []
        back_area_x = []
        # 保护高度和吊具是否载箱以及箱子尺寸有关
        if self.is_spreader_load:
            if self.container_size == 1:  # 2596
                protected_height = self.spreader_height - 2.591
            else:  # 其他情况默认为高箱
                protected_height = self.spreader_height - 2.896
        else:
            protected_height = self.spreader_height - 0.2
        # 保护长度区域在12米
        protected_length = 12

        # 小车朝司机室前方向点云截取
        for index in range(len(self.lidar_all_x)):
            if -protected_length < self.lidar_all_x[index] < -2 and \
                    protected_height - 0.5 < self.lidar_all_y[index] < self.config['minGantryHeight'] - 0.5:
                head_area_x.append(self.lidar_all_x[index])

        # 小车朝司机室后方向点云截取
        for index in range(len(self.lidar_all_y)):
            if 2 < self.lidar_all_x[index] < protected_length and \
                    protected_height - 0.5 < self.lidar_all_y[index] < self.config['minGantryHeight'] - 0.5:
                back_area_x.append(self.lidar_all_x[index])
        # 二次校验，防止噪点干扰
        # 前方检验
        result = self.double_check(head_area_x)
        if result:
            self.head_barrier = abs(result[1])
            self.warning_flag_F = True
            self.msg_fmt["bannerDis1"] = self.head_barrier
        else:
            self.warning_flag_F = False
            self.head_barrier = 99
        # 后方校验
        result = self.double_check(back_area_x)
        if result:
            self.back_barrier = abs(result[0])
            self.warning_flag_B = True
            self.msg_fmt["bannerDis2"] = self.back_barrier
        else:
            self.back_barrier = 99
            self.warning_flag_B = False
        self.msg_fmt["hasBanner"] = "有障碍物" if (self.warning_flag_F or self.warning_flag_B) else "无障碍"

    @staticmethod
    def double_check(protected_area_x):
        # 检测区域内超过3个点，并且点与点之间的间隔小于0.5米，就认为是集装箱，排除干扰点
        if len(protected_area_x) > 5:
            protected_area_x = sorted(protected_area_x)
            cnt = 0
            for index in range(len(protected_area_x) - 1):
                if abs(protected_area_x[index + 1] - protected_area_x[index]) < 0.5:
                    cnt += 1
            if cnt > 3:
                return min(protected_area_x), max(protected_area_x)
        return None

    def is_spreader_safe(self):
        # 吊具下降过程中检查当前速度能否造成砸箱危险
        if self.is_spreader_down:
            # 当前排集装箱的高度

            self.get_current_row()
            if self.current_row == -1:
                current_height = 0
            else:
                current_height = self.row_height[self.current_row]
            # 根据当前的高度计算出来的刹车距离
            # s = （v - v0）² / 2a, at = v - v0
            distance = (self.current_speed * self.current_speed * self.rated_time
                        * self.rated_speed) / 1200000
            # 保护高度和吊具是否载箱以及箱子尺寸有关
            if self.is_spreader_load:
                if self.container_size == 1:  # 2596
                    protected_height = self.spreader_height - 2.591
                else:  # 其他情况默认为高箱
                    protected_height = self.spreader_height - 2.896
            else:
                protected_height = self.spreader_height - 0.2
            # 减速距离小于零界值，报警
            # current_height + distance = protect_height + 0.3
            if protected_height - current_height - 0.3 <= distance:
                self.spreader_warning = True
                self.msg_fmt["thisH"] = round(current_height, 3)
                self.msg_fmt["spreaderSpeed"] = self.current_speed
                self.msg_fmt["lowDis"] = round(distance, 3)
            else:
                self.spreader_warning = False
            self.msg_fmt["inDanger"] = "防砸加速" if self.spreader_warning else "否"

    def get_current_row(self):
        # 获取当前的小车排位值：0 到 16，如果在鞍梁或者装车道，用-1标记出来
        # 取小车前部端梁为参考
        if self.trolley_location < self.config['rowData'][0] - 1.4:  # 坐标轴左侧装车道
            self.current_row = -1
        elif self.trolley_location > self.config['rowData'][self.config["rowNumber"] - 1] + 1.4:  # 坐标轴右侧装车道
            self.current_row = -1
        # 在坐标轴左侧鞍梁位置
        elif self.config['rowData'][self.config["leftSaddle"]] + 1.4 < self.trolley_location < self.config["rowData"][
            self.config["leftSaddle"] + 1] - 1.4:
            self.current_row = -1
        # 在坐标轴右侧鞍梁位置
        elif self.config['rowData'][self.config["rightSaddle"] - 2] + 1.4 < self.trolley_location < \
                self.config['rowData'][self.config["rightSaddle"] - 1] - 1.4:
            self.current_row = -1
        else:
            # 取距离小车中心最近的箱中心点索引即排位
            temp = [abs(self.config["rowData"][i] - self.trolley_location) for i in range(self.config["rowNumber"])]
            self.current_row = temp.index(min(temp))

    @staticmethod
    def sortFirstElement(x, y):
        newtuple = [(x, y) for x, y in zip(x, y)]

        output_x = []
        output_y = []
        for ele in sorted(newtuple):
            output_x.append(ele[0])
            output_y.append(ele[1])

        return output_x, output_y

    def merge(self):
        if self.config["beamPosition"] > 0:
            if self.config["beamLidar"] > 0:
                i = 0
                while i < len(self.lidarF_x):
                    if self.lidarF_x[i] < -2:
                        del self.lidarF_x[i]
                        del self.lidarF_y[i]
                    else:
                        i += 1
                i = 0
                while i < len(self.lidarB_x):
                    if self.lidarB_x[i] > 2:
                        del self.lidarB_x[i]
                        del self.lidarB_y[i]
                    else:
                        i += 1
            else:
                i = 0
                while i < len(self.lidarF_x):
                    if self.lidarF_x[i] > 2:
                        del self.lidarF_x[i]
                        del self.lidarF_y[i]
                    else:
                        i += 1
                i = 0
                while i < len(self.lidarB_x):
                    if self.lidarB_x[i] < -2:
                        del self.lidarB_x[i]
                        del self.lidarB_y[i]
                    else:
                        i += 1
        else:
            if self.config["beamLidar"] > 0:
                i = 0
                while i < len(self.lidarF_x):
                    if self.lidarF_x[i] > 2:
                        del self.lidarF_x[i]
                        del self.lidarF_y[i]
                    else:
                        i += 1
                i = 0
                while i < len(self.lidarB_x):
                    if self.lidarB_x[i] < -2:
                        del self.lidarB_x[i]
                        del self.lidarB_y[i]
                    else:
                        i += 1
            else:
                i = 0
                while i < len(self.lidarF_x):
                    if self.lidarF_x[i] < -2:
                        del self.lidarF_x[i]
                        del self.lidarF_y[i]
                    else:
                        i += 1
                i = 0
                while i < len(self.lidarB_x):
                    if self.lidarB_x[i] > 2:
                        del self.lidarB_x[i]
                        del self.lidarB_y[i]
                    else:
                        i += 1

    def start(self):
        print_count = 0
        being_time = time()
        except_count = 0
        loop_time_count = 0
        while True:
            try:
                self.start_time = time()
                self.internal_error = 0
                if self.scan_counter > self.config["scanConfigRotation"]:
                    try:
                        self.lidarB.sendHeartBeat()
                    except:
                        self.lidarB_state = False
                        print('司机室后扫描仪出现故障')
                    else:
                        self.lidarB_state = True
                    try:
                        self.lidarF.sendHeartBeat()
                    except:
                        self.lidarF_state = False
                        print('司机室前扫描仪出现故障')
                    else:
                        self.lidarF_state = True

                    self.config = self.load_cfg()
                    self.msg_fmt["deviceID"] = self.config["deviceID"]

                    self.lidarB.setXOffset(self.config["BXoffset"] + self.config["overallXoffset"])
                    self.lidarB.setYOffset(self.config["BYoffset"] + self.config["overallYoffset"])
                    self.lidarB.setThetaOffset(self.config["BThetaoffset"] + self.config["overallThetaoffset"])
                    self.lidarB.setXReverse(self.config["BXReverse"])

                    self.lidarF.setXOffset(self.config["FXoffset"] + self.config["overallXoffset"])
                    self.lidarF.setYOffset(self.config["FYoffset"] + self.config["overallYoffset"])
                    self.lidarF.setThetaOffset(self.config["FThetaoffset"] + self.config["overallThetaoffset"])
                    self.lidarF.setXReverse(self.config["FXReverse"])

                    self.scan_counter = 0
                else:
                    self.scan_counter += 1

                time2 = time()

                # 检查扫描仪心跳是否正常，否则重启程序
                self.is_lidar_alive()

                self.is_lidarB_dirty = self.lidarB.getDirty()
                self.is_lidarF_dirty = self.lidarF.getDirty()

                lidarB_x, lidarB_y = self.lidarB.getXY()
                lidarF_x, lidarF_y = self.lidarF.getXY()
                # print('拿到的数据点长度：', len(FM_1_x))

                time3 = time()

                self.lidarB_x = list(lidarB_x)
                self.lidarB_y = list(lidarB_y)
                self.lidarF_x = list(lidarF_x)
                self.lidarF_y = list(lidarF_y)

                time4 = time()

                # # 计算吊具高度
                self.cal_spreader_height()

                time5 = time()

                # 计算小车位置
                self.cal_trolley_location(
                    self.config["beamPosition"],
                    self.config["minGantryHeight"],
                    self.config["maxGantryHeight"])

                time6 = time()
                # 将两台扫描仪进行切割，吊具前方的扫描仪切掉吊具后方的数据，后方的扫描仪切掉前方的数据。
                self.merge()
                # 合并两台扫描数据
                self.lidar_all_x = self.lidarB_x + self.lidarF_x
                self.lidar_all_y = self.lidarB_y + self.lidarF_y
                self.lidar_all_x, self.lidar_all_y = self.sortFirstElement(self.lidar_all_x, self.lidar_all_y)

                time7 = time()

                # 切割不需要的点
                self.cut_interference()

                # # 读取PLC发送过来的信息
                if self.config['PLCSwitch']:
                    self.plc_recv()

                # 计算每一排的箱高
                self.cal_row_height()

                time8 = time()

                # 检查小车运行方向是否有障碍物
                self.is_trolley_safe()

                # 检查吊具下降是否安全
                self.is_spreader_safe()

                time9 = time()

                if self.config['PLCSwitch']:
                    try:
                        send_msg = self.format_send_msg()
                        self.plc_send(send_msg)
                    except Exception as plc_error:
                        self.log.error('plc 发送数据失败 :{}'.format(traceback.format_exc()))
                        self.plc_state = False
                    else:
                        self.plc_state = True

                if not self.lidarB_state:
                    try:
                        self.lidarB.joinThread()
                        self.lidarB.disconnect()
                        self.initBlidar()
                    except Exception:
                        self.lidarB_state = False
                        self.log.warning("防打保龄司机室后侧扫描仪断线，重连失败:{}".format(traceback.format_exc()))
                        self.lidarF_reconnect_count += 1
                    else:
                        self.lidarB_state = True
                        self.lidarF_reconnect_count = 0

                if not self.lidarF_state:
                    try:
                        self.lidarF.joinThread()
                        self.lidarF.disconnect()
                        self.initFlidar()
                    except Exception:
                        self.lidarF_state = False
                        self.log.warning("防打保龄司机室前侧扫描仪断线，重连失败:{}".format(traceback.format_exc()))
                        self.lidarB_reconnect_count += 1
                    else:
                        self.lidarF_state = True
                        self.lidarB_reconnect_count = 0

                if not self.plc_state:
                    try:
                        self.plc.keeprecving = False
                        self.plc.joinThread()
                        self.plc.plc_disconnect()
                        self.initPLC()
                    except Exception:
                        self.log.error("PLC断线重连失败:{}".format(traceback.format_exc()))
                        self.plc_state = False
                        self.plc_reconnect_count += 1
                    else:
                        self.plc_state = True
                        self.plc_reconnect_count = 0

                # 任意一个重连次数达到最大值，都将重启程序
                if self.plc_reconnect_count > 10 or self.lidarB_reconnect_count > 10 or \
                        self.lidarF_reconnect_count > 10 or self.plc_msg_count > 10:
                    self.log.error('重连次数达到最大值，重启程序')
                    self.__del__()
                    sys.exit(0)

                time10 = time()

                self.plot()

                time11 = time()

                if self.config['PLCSwitch']:
                    # 看门狗喂狗
                    try:
                        self.FD.feed()
                    except Exception:
                        #  看门狗关闭，程序退出
                        self.__del__()
                        sys.exit(-1)

                now_time = time()
                loop_time = now_time - self.start_time

                # 检测单次循环时间，如果超过500ms，记录本次数据
                if loop_time > 0.5:
                    m_time = [self.start_time, time2, time3, time4, time5, time6, time7, time8, time9, time10, time11]
                    new_m_time = [int((m_time[index + 1] - m_time[index]) * 1000) for index in range(len(m_time) - 1)]
                    self.log.warning('循环用时 :{}ms, time Gap:{} '.format(loop_time, str(new_m_time)))
                    print('循环时间超时！！！！！！！！！！！！---->:', loop_time)
                    if loop_time > 2:
                        loop_time_count += 1
                    else:
                        loop_time_count = 0
                    if loop_time_count > 15:
                        self.log.error('循环超时次数达到最大值，重启程序！')
                        self.__del__()
                        sys.exit(0)
                if print_count > 8:
                    m_time = [self.start_time, time2, time3, time4, time5, time6, time7, time8, time9, time10, time11]
                    new_m_time = [int((m_time[index + 1] - m_time[index]) * 1000) for index in range(len(m_time) - 1)]
                    print_count = 1
                    used_time = round(((now_time - being_time) / 10) * 1000, 2)  # 单位ms
                    being_time = now_time
                    print(STDOUT_FMT.format(**self.msg_fmt))
                    print('循环用时 :{}ms, time Gap:{} '.format(used_time, str(new_m_time)))
                else:
                    print_count += 1

            except KeyboardInterrupt:
                self.__del__()
                sys.exit(0)
            except Exception:
                except_count += 1
                self.log.warning('捕捉到异常，异常计数：{}，{}'.format(except_count, traceback.format_exc()))
            else:
                except_count = 0
            finally:
                if except_count > 15:
                    self.log.error('异常次数连续达到最大值，重启程序')
                    self.__del__()
                    sys.exit(0)


if __name__ == "__main__":
    try:
        obj = SPSSytem()
        obj.start()
    except Exception as error:
        traceback.print_exc()
        sys.exit(0)
