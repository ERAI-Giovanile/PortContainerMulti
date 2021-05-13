
# from math import sin,cos,radians,log
# from debug_server import main
# from time import sleep
# import json
# number = 100
# num = -30
# obj = main()
# _s = 1
# while True:
#     if num > 50:
#         num = -20
#     else:
#         num += 5
#     sleep(0.1)
#     x1 = [num+ele/10 for ele in range(number)]
#     y1 = [sin(radians((ele/10) * 10)) for ele in range(number)]
#
#     x2 = [ele/10+num for ele in range(number)]
#     y2 = [cos(ele / 10)+3 for ele in range(number)]
#
#     config = {}
#     with open('spsconfig.json', 'r') as f:
#         config = json.load(f)
#     obj.set_debug_draw_data(x1,y1,x2,y2)
#     obj.set_config(config)

import numpy as np
from struct import unpack,pack
import matplotlib.pyplot as plt


with open('error-11-29.txt', 'r') as file:
    while True:
        data = file.readline()
        if not data:
            break
        time_stamp = data
        print('time:', time_stamp)
        data = file.readline()
        try:
            x = [float(ele) for ele in data.strip('[]\n').split(',')]
        except Exception as err:
            print(err)
            x = []
        data = file.readline()
        y = [float(ele) for ele in data.strip('[]\n').split(',')]
        plt.clf()
        # plt.axis([-2,2,-3,20])
        # plt.axis([-1.5,1,-1,20])
        plt.title(time_stamp)
        plt.plot(x, y, 'b.')
        plt.plot([0, 0], [-0.3, 24], '-')  # 显示中心线
        plt.plot([-2, 2], [-1.78, -1.78], '-')
        plt.draw()
        plt.pause(0.001)

        temp = []
        for i in range(len(x)):
            if -1.4 < x[i] < 1.5 and 0.2 < y[i] < 20:
                temp.append(y[i])
        value = np.median(sorted(temp)[-int(len(temp) * 0.3):]) - 2.05
        print('通过xy计算出来的吊具高度' ,value)
        input()

        # temp_y
        data = file.readline()
        if not data:
            break
        time_stamp = data
        print('time:', time_stamp)
        data = file.readline()
        temp_y = file.readline()
        temp_y = [float(ele) for ele in temp_y.strip('[]\n').split(',')]
        # print(x)
        # input()
        # temp = sorted(temp_y)
        # print('length:',len(temp), len(temp)*0.3, int(len(temp)*0.3),-int(len(temp)*0.3) )
        # temp = temp[-int(len(temp)*0.3):]
        # print('before the median : ', temp)
        # temp = np.median(temp) - 2.05
        print('temp_y 获得的吊具高度值：',np.median(sorted(temp_y)[-int(len(temp_y) * 0.3):]) - 2.05)
        input()

        # RSSI 高亮点
        time_stamp = file.readline()
        print(time_stamp, "RSSI")
        RSSI_x=file.readline()
        RSSI_y=file.readline()
        try:
            RSSI_x = [float(ele) for ele in RSSI_x.strip('[]\n').split(',')]
            RSSI_y = [float(ele) for ele in RSSI_y.strip('[]\n').split(',')]
            print('高亮点计算出来的吊具高度：',np.median(RSSI_y) - 2.05)
        except Exception:
            print('没有高亮点！！！')
        else:
            plt.clf()
            # plt.axis([-2, 2, -3, 20])
            plt.title(time_stamp)
            plt.plot(RSSI_x, RSSI_y, 'b.')
            # plt.plot([0, 0], [-0.3, 24], '-')  # 显示中心线
            # plt.plot([-2, 2], [-1.78, -1.78], '-')
            plt.draw()
            plt.pause(0.001)

        input()














