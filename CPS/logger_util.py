# -*- encoding: utf-8 -*-
"""
@Author  :   VictorWong
@License :   (C) Copyright 2013-2019, Wuhan Guide Electric Technology com.,ltd.
@Contact :   victorwangms@163.com
@Software:   PyCharm
@File    :   logger_util.py
@Time    :   2019/4/23 14:28
@Desc    :
"""
# import logging
# import os


# class UniversalLogger(object):
#     """
#     通用日志类, basedir是需要输入日志的目录
#     """
#
#     def __init__(self, base_dir):
#         self.base_dir = base_dir
#         self.log_path = {
#             logging.ERROR: os.path.abspath(os.path.join(base_dir, 'error', 'error.log')),
#             logging.CRITICAL: os.path.abspath(os.path.join(base_dir, 'critical', 'critical.log')),
#             logging.DEBUG: os.path.abspath(os.path.join(base_dir, 'debug', 'debug.log')),
#             logging.WARNING: os.path.abspath(os.path.join(base_dir, 'warning', 'warning.log')),
#             logging.INFO: os.path.abspath(os.path.join(base_dir, 'info', 'info.log')),
#         }
#
#         for path in self.log_path.values():
#             if not os.path.exists(path):
#                 os.makedirs(os.path.dirname(path))
#
#         self.__loggers = {}
#         logLevels = self.log_path.keys()
#         format_general = logging.Formatter('%(asctime)s(%(levelname)s) %(name)s: %(message)s.')
#         format_error = logging.Formatter(
#             '%(asctime)s(%(levelname)s) %(filename)s-%(module)s-%(funcName)s: %(message)s.')
#         for level in logLevels:
#             # 创建logger
#             logger = logging.getLogger(str(level))
#             logger.setLevel(level)
#             # 创建hander用于写日日志文件
#             log_path = os.path.abspath(self.log_path[level])
#             fh = logging.FileHandler(log_path)
#             ch = logging.StreamHandler()
#             # 定义日志的输出格式
#             if level == logging.ERROR:
#                 fh.setFormatter(format_error)
#                 ch.setFormatter(format_error)
#             else:
#                 fh.setFormatter(format_general)
#                 ch.setFormatter(format_general)
#             fh.setLevel(level)
#             ch.setLevel(level)
#             # 给logger添加hander
#             logger.addHandler(fh)
#             logger.addHandler(ch)
#             self.__loggers.update({level: logger})
#
#     def info(self, message):
#         self.__loggers[logging.INFO].info(message)
#
#     def error(self, message):
#         self.__loggers[logging.ERROR].error(message, exc_info=True)
#
#     def warning(self, message):
#         self.__loggers[logging.WARNING].warning(message, exc_info=True)
#
#     def debug(self, message):
#         self.__loggers[logging.DEBUG].debug(message)
#
#     def critical(self, message):
#         self.__loggers[logging.CRITICAL].critical(message, exc_info=True)
import logging
import os
import time
import traceback
from logging.handlers import TimedRotatingFileHandler

base_dir = os.path.abspath(os.getcwd())


class UniversalLogger(object):
    def __init__(self, logger_name):
        self.ymd_time = time.strftime('%Y%m%d', time.localtime(time.time()))
        self.__log_path = os.path.join(base_dir, 'logs', self.ymd_time, logger_name, 'running.log')
        if not os.path.exists(self.__log_path):
            os.makedirs(os.path.dirname(self.__log_path))
        self.logger = logging.getLogger(logger_name)
        self.logger.setLevel(logging.INFO)
        self.fmt_general = logging.Formatter(
            '%(asctime)s(%(levelname)s) %(filename)s(%(module)s)[%(funcName)s|%(lineno)d]: %(message)s')
        self.fmt_special = logging.Formatter(
            '%(asctime)s(%(levelname)s) %(filename)s(%(module)s)[%(funcName)s|%(lineno)d]: %(message)s')
        self.__ch = None
        # 定时创建新滚动日志的单位
        self.__log_unit = "H"
        # 滚动创建日志的时间长度
        self.__log_interval = 1
        # 允许存在的滚动创建的日志最大数量，多余旧日志将被删除
        self.__log_backupcnt = 72
        # 定时删除日志滚动模块
        self.__fh = None
        self.__add_stdout_handler()
        self.__add_fileout_handler()

    def get_log_name_fmt(self):
        return self.__log_path

    def get_log_unit(self):
        return self.__log_unit

    def get_log_interval(self):
        return self.__log_interval

    def get_log_backupcnt(self):
        return self.__log_backupcnt

    def _set_log_name_fmt(self, fmt):
        self.__log_path = fmt

    def _set_log_unit(self, unit):
        self.__log_unit = unit

    def _set_log_interval(self, interval):
        self.__log_interval = interval

    def _set_log_backupcnt(self, cnt):
        self.__log_backupcnt = cnt

    def __add_stdout_handler(self):
        self.__ch = logging.StreamHandler()
        self.__ch.setFormatter(self.fmt_special)
        self.logger.addHandler(self.__ch)

    def __add_fileout_handler(self):
        self.__fh = TimedRotatingFileHandler(self.__log_path, when=self.__log_unit, interval=self.__log_interval,
                                             backupCount=self.__log_backupcnt)
        self.__fh.setFormatter(self.fmt_general)
        self.logger.addHandler(self.__fh)

    def _reset_fileout_handler(self):
        self.logger.removeHandler(self.__fh)
        self.__fh = None
        self.__add_fileout_handler()


if __name__ == '__main__':
    logger_f = UniversalLogger('Chassis logger')
    logger = logger_f.logger
    logger_f._set_log_unit("S")
    logger_f._set_log_interval(1)
    logger_f._set_log_backupcnt(12)
    logger_f._reset_fileout_handler()
    logger.info('bbb')
    logger.debug('aa')
    logger.critical('cri')
    for i in range(1000000):
        try:
            1 / 0
        except Exception:
            logger.error('aaaaaa' + traceback.format_exc())
