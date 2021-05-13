#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""
@Time : 2020/4/21 12:33
@License :   (C) Copyright 2013-2019, Wuhan Guide Electric Technology com.,ltd.
@Author : VictorWong
@Contact :   victorwangms@163.com
@File : base_sqlite.py
@Description: None
"""
import sqlite3


class SqliteDB(object):
    def __init__(self, db_path):
        self.db = None
        self.db_path = db_path
        self.cursor = None

    def connect_db(self, timeout=1):
        self.db = sqlite3.connect(self.db_path, timeout)
        self.cursor = self.db.cursor()

    def disconnect_db(self):
        self.db.close()

    def commit_db(self):
        self.db.commit()

    def execute_db(self, sql):
        self.cursor.execute(sql)
        self.commit_db()


if __name__ == '__main__':
    db = SqliteDB("temp.db")
    db.connect_db()
    sql = """create table if not exists Error
            (No integer primary key autoincrement, temp varchar(50));"""
    db.execute_db(sql)
    db.commit_db()
    sql = """insert into Error(temp) values("我是谁，我在哪")"""
    db.execute_db(sql)
    sql = """insert into Error(temp) values("我是谁，我在哪")"""
    db.execute_db(sql)
    db.commit_db()
    db.disconnect_db()
