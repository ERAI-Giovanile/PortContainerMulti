#!/usr/bin/env python3
# CODING:UTF-8
# _*_ @Author:Amee _*_
# _*_ @Date:2019 - - -  _*_


from socket import socket, AF_INET, SOCK_STREAM
import os


class FeedWatchDog(object):
    def __init__(self, port):
        self.s = socket(AF_INET, SOCK_STREAM)
        self.s.connect(('localhost', port))

        clientPID = os.getpid()
        self.msg = ('SPSclientPID:'+str(clientPID)).encode()

    def feed(self):
        self.s.send(self.msg)

    def close(self):
        self.s.close()

    def test(self):
        while True:
            try:
                self.feed()
                input('feed dog now :\n')
            except Exception:
                self.close()
                os._exit(1)


if __name__ == "__main__":
    obj = FeedDog(8889)
    obj.test()