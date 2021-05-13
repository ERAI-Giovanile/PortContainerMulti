import socket
import threading
import time
from struct import unpack


class PLC_TCP(object):
    def __init__(self, ip, port, plc_id):
        self.ip = ip
        self.port = port
        self.plc_id = plc_id
        self.recv = b''
        self.plc_socket = socket.socket(socket.AF_INET,
                                        socket.SOCK_STREAM)
        self.t = threading.Thread(target=self.plc_recv)
        self.lock = threading.RLock()
        self.keeprecving = False

    def plc_connect(self):
        self.plc_socket.connect((self.ip, self.port))
        self.keeprecving = True
        self.t.start()

    def plc_disconnect(self):
        self.plc_socket.close()

    def joinThread(self):
        self.keeprecving = False
        self.t.join()

    def plc_recv(self):
        try:
            while self.keeprecving:
                self.lock.acquire()
                self.recv = self.plc_socket.recv(32)
                self.lock.release()
            # print(self.recv)
        except Exception as e:
            print(e)
            # self.plc_disconnect()

    def plc_send(self, send_data):
        self.plc_socket.sendall(send_data)

    def get_recv(self):
        if self.recv:
            trolley_location = unpack('>l', self.recv[4:8])[0] / 1000  # 小车位置 单位m
            spreader_height = unpack('>H', self.recv[8:10])[0] / 1000  # 编码器吊具高度，单位m，十六进制
            # 如果开闭锁没有发过来（0x00状态），默认为闭锁带箱
            spreader_lock = False if unpack('>B', self.recv[10:11])[0] == 1 else True  # 开闭锁状态 开锁：0x01  闭锁0x02
            is_rtg_moving = False if unpack('>B', self.recv[11:12])[0] == 3 else True
            # 大车状态 停止：0x03, 其他状态认为时大车跨呗状态
            trolley_direction = unpack(">B", self.recv[12:13])[0]  # 小车运行方向 0x03：停止 0x01：山-》水 0x02：水-》山
            is_spreader_down = True if unpack('>B', self.recv[13:14])[0] == 2 else -1 if unpack('>B', self.recv[13:14])[0] == 1 else False
            # 吊具是否下降 0x03:停止 0x01：向上 0x02：向下
            rated_slowdown_time = unpack('>f', self.recv[14:18])[0]  # 吊具起升减速时间，单位s
            rated_speed = unpack('>f', self.recv[18:22])[0]  # 吊具起升额定速度，单位为m/min
            current_speed = unpack('>f', self.recv[22:26])[0]  # 起升速度，0~100%
            spreader_size = unpack('>B',self.recv[26:27])[0]  # 吊具的尺寸
            return [trolley_location, spreader_height, spreader_lock, is_rtg_moving, trolley_direction,
                    is_spreader_down, current_speed, rated_slowdown_time, rated_speed,spreader_size]
        else:
            return []


if __name__ == '__main__':
    test = PLC_TCP('172.1.1.155', 2000, 1)
    data = [1, 3]
    rec = b''
    test.plc_connect()
    print(data)
    while 1:
        # print('aaaa')
        a = test.get_recv()

        # rec_plc_data = [struct.unpack('>H', ele) for ele in rec]
        print(a)

        time.sleep(1)
