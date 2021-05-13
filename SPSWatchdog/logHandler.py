#!/usr/bin/python3

'''
日志监测模块功能如下：
1，log按天切分，每天一个log文件
2, 保留三天的log文件，过期删除，
3，每一个log文件做大小限制，超过则删除。
'''
import datetime
from threading import Thread
import os
from time import sleep, time, mktime


# fileName = self.logPrefix + now + '.txt'  self.logPrefix = 'xydatalog'
class LogHandler(object):
    def __init__(self, expired, maxSize, interval):
        """
        :param expired:  过期时间  Day
        :param maxSize:  存储日志文件最大尺寸 MB
        interval : 间隔时间后再次检测, minute
        """
        self.expired = expired
        self.maxSize = maxSize
        self.interval = interval * 60
        self.thread = Thread(target=self.threadHandler)
        self.loop = True

    def threadHandler(self):
        while self.loop:
            now = datetime.datetime.now()
            time_offset = datetime.timedelta(days=-3)
            delete_time = mktime((now + time_offset).timetuple())
            if int(now.strftime("%H")) == 20:
                for i in os.listdir('./'):
                    if 'txt' in i:
                        file_time = os.path.getctime(i)
                        if file_time < delete_time:
                            try:
                                os.remove(os.path.join(os.getcwd(), i))
                            except Exception as e:
                                print('remove failed and retry... ', e)
                                sleep(2)
                        else:
                            if os.path.getsize(i) / 1024 / 1024 > self.maxSize:
                                try:
                                    os.remove(os.path.join(os.getcwd(), i))
                                except Exception as e:
                                    print('remove failed and retry... ', e)
                                    sleep(2)
            sleep(self.interval)
            #         log_list = []
            #         normal_list = []
            #         # 生成过期时间内的日志文件名
            #         for day_count in range(self.expired):
            #             normal_list.append(
            #                 self.logPrefix + (now - timedelta(days=day_count)).strftime('-%m-%d') + '.txt')
            #         # print(normal_list)
            #         app_list = os.listdir('.')
            #         # 查找所有文件名
            #         for ele in app_list:
            #             if self.logPrefix in ele:
            #                 log_list.append(ele)
            #
            #         # 删除过期的日志
            #         for ele in log_list:
            #             if ele not in normal_list:
            #                 os.remove(ele)
            #                 log_list.remove(ele)
            #
            #         # 删除大小超界日志文件
            #         # for ele in log_list:
            #         #     # 单位大小为MB
            #         #     if int(os.path.getsize(ele)) / 1024 / 1024 > self.maxSize:
            #         #         os.remove(ele)
            #         sleep(self.interval)
            #
            # except Exception as e:
            #     print('remove failed and retry... ', e)
            #     sleep(2)

    def start(self):
        self.thread.start()

    def endThread(self):
        self.loop = False
        self.thread.join()

    def test(self):
        now = datetime.now().strftime('-%m-%d')
        fileName = self.logPrefix + now + '.txt'

        open('xydatalog-07-09.txt', 'a')
        open('xydatalog-07-08.txt', 'a')
        open('xydatalog-07-07.txt', 'a')
        open('xydatalog-07-06.txt', 'a')
        print('ready !!!!!!!!!!!!!!')
        self.thread.start()
        try:
            while True:
                with open('xydatalog-07-10.txt', 'a') as file:
                    msg = str([ele for ele in range(100000)])
                    file.write(msg)
                sleep(0.1)
        except KeyboardInterrupt:
            self.loog = False
            self.thread.join()
            os._exit(0)
        except Exception as e:
            print(e)
            sleep(2)
            print('关闭失败，重新来过。。。。')


if __name__ == "__main__":
    # now = datetime.now().strftime('-%m-%d')
    # fileName = 'xydatalog' + now + '.txt'
    # open(fileName, 'a')
    obj = LogHandler(3, 200, 1)
    obj.start()