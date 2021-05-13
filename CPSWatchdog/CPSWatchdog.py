#!/usr/bin/env python3
# CODING:UTF-8
# _*_ @Author:Amee _*_
# _*_ @Date:2019 - - -  _*_

from socket import socket, AF_INET, SOCK_STREAM, error

from time import sleep, time, strftime, localtime
import re
import os, sys
import platform

PLATFORM = platform.system()
if PLATFORM == "Windows":
    import win32api
from logHandler import LogHandler

DIR = ""
if PLATFORM == "Linux":
    DIR = r"/home/{}/CPS/".format(os.environ["USERNAME"])

class WatchDog(object):
    def __init__(self, port, fileName, time_out=10):
        self.dogSocket = socket(AF_INET, SOCK_STREAM)
        self.dogSocket.settimeout(1)
        self.dogSocket.bind(('localhost', port))
        self.dogSocket.listen(5)
        self.timeOut = time_out
        self.fileName = fileName
        if PLATFORM == "Windows":
            if os.path.exists(self.fileName + ".exe"):
                self.fileName += ".exe"
            else:
                self.fileName += ".py"
        elif PLATFORM == "Linux":
            if not os.path.exists(os.path.join(DIR, self.fileName)):
                self.fileName += ".py"

    def waitFood(self):
        '等待主进程喂狗,非阻塞式'
        outer_starttime = time()
        if PLATFORM == "Windows":
            if self.fileName.endswith("exe"):
                win32api.ShellExecute(0, 'open', self.fileName, '', '', 1)
            else:
                win32api.ShellExecute(0, 'open', 'python.exe', self.fileName, '', 1)
        elif PLATFORM == "Linux":
            if self.fileName.endswith("py"):
                os.system("cd %s && gnome-terminal -- python3 %s" % (DIR, os.path.join(DIR, self.fileName)))
            else:
                os.system("cd %s && gnome-terminal -- %s" % (DIR, os.path.join(DIR, self.fileName)))
        while True:
            try:
                conn, addr = self.dogSocket.accept()
                if conn:
                    print('client connected', addr)
                    conn.settimeout(1)
                    starttime = time()
                    clientPID = ''
                    while True:
                        try:
                            recv = conn.recv(1024)
                            # print('recv data :', recv.decode())
                            if not recv:
                                break
                        except error as e:
                            e = str(e)
                            if e == 'timed out':
                                print('time out for receiving PID')
                            else:
                                print('program exits ：', e)
                                sleep(1)
                                break
                        except Exception as e:
                            print('捕获到其他的异常：', e)
                            sleep(1)
                            break
                        else:
                            recv = recv.decode()
                            starttime = time()
                            # print('有数据来，更新时间', starttime, recv)
                            try:
                                clientPID = re.findall('\D(\d+)', recv)[0]
                            except Exception as e:
                                pass
                                # print('查找出现异常：', e)
                            else:
                                print('CPS Client PID : {}'.format(clientPID))
                        finally:
                            sleep(1)
                            if time() - starttime > self.timeOut and clientPID:
                                print(' now kill kill and restart the target exe ')
                                if PLATFORM == "Windows":
                                    os.popen('taskkill /F /PID {}'.format(clientPID))
                                elif PLATFORM == "Linux":
                                    os.system("gnome-terminal -- kill -9 %s" % clientPID)
                                print('kill the target successfully')
                                sleep(1)
                                break
            except error as err:
                e = str(err)
                if e == 'timed out':
                    print('没有设备连接进来，waiting。。。')
                else:
                    print('restart script now', err)
                    with open('cpsError.txt', 'a') as filewriter:
                        timeStamp = strftime("%Y-%m-%d %H:%M:%S", localtime())
                        filewriter.write(str(e) + ' ,watchdogrestart ' + timeStamp + ' ,\n')
                    sleep(1)
                    # restart
                    python = sys.executable
                    os.execl(python, python, *sys.argv)
            except Exception as e:
                e = str(e)
                # print('outer error',e)
                with open('cpsError.txt', 'a') as filewriter:
                    timeStamp = strftime("%Y-%m-%d %H:%M:%S", localtime())
                    filewriter.write(str(e) + ' ,watchdogrestart ' + timeStamp + ' ,\n')
                sleep(1)
                python = sys.executable
                os.execl(python, python, *sys.argv)
            finally:
                if time() - outer_starttime > 15:
                    # 超时将自动启动程序
                    if PLATFORM == "Windows":
                        if self.fileName.endswith("exe"):
                            win32api.ShellExecute(0, 'open', self.fileName, '', '', 1)
                        else:
                            win32api.ShellExecute(0, 'open', 'python.exe', self.fileName, '', 1)
                    elif PLATFORM == "Linux":
                        if self.fileName.endswith("py"):
                            os.system("cd %s && gnome-terminal -- python3 %s" % (DIR, os.path.join(DIR, self.fileName)))
                        else:
                            os.system("cd %s && gnome-terminal -- %s" % (DIR, os.path.join(DIR, self.fileName)))
                    # win32api.ShellExecute(0, 'open',  self.fileName, '', '', 1)
                    print('start the target successfully...')
                    outer_starttime = time()


if __name__ == '__main__':
    expired = 3
    # 日志文件最大500MB
    maxSize = 500
    # 轮训间隔时间：1分钟
    interval = 60
    # 建立日志对象
    logobj = LogHandler(expired, maxSize, interval)
    # 启动日志监测线程
    logobj.start()
    # 设置程序死机等待多少秒时间后重启
    restartTime = 20
    # 需要启动的目标文件名
    targetFileName = 'cps'
    # 设置通信端口号
    port = 9998
    obj = WatchDog(port, targetFileName, restartTime)
    obj.waitFood()
