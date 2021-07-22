#!/usr/bin/python
# -*- coding: utf-8 -*-
# license removed for brevity

# pythonでROSのソフトウェアを記述するときにimportするモジュール
import rospy

import sqlite
# 自分で定義したmessageファイルから生成されたモジュール
from arc2020.msg import sql

CYCLES = 1 #処理周波数
# 定数などの定義ファイルimport


# class sampleDB 定義
class sampleDB(object):
    """
    sample DBクラス
    """
#--------------------
# コンストラクタ
    def __init__(self):
        """
        コンストラクタ
        """
        self.cyclecount = 0
        # 受信作成
        self.sub_client  = rospy.Subscriber('sql', sql, self.insertCallback, queue_size=1)
        # messageのインスタンスを作る
        self.msg_sql = sql()
        # dbを作成する
        db_name = 'yasai'
        self.db = sqlite.sqlite(db_name) 
        print("db create by sampleDB")

        ##self.db.selectTable("yasai")
        self.db.selectyasai1()
        # tableを作成する
        # self.table_seed = 'seed'
        # self.sqlite.createTable(self.table_seed)

#--------------------
# 受信コールバック
    def insertCallback(self, msg):
        """
        brainの受信コールバック
        """
        print("table " + msg.table + ",name " + msg.name)

        # msg.tableにmsg.nameを登録する       
        self.db.insert(msg.table,msg.name)
#--------------------
#    def main(self):
        # 特に処理なし

def sampleDB_py():
    # 初期化宣言 : このソフトウェアは"sampleDB_py_node"という名前
    rospy.init_node('sampleDB_py_node', anonymous=True)
    # インスタンスの作成 
    sample_DB = sampleDB()
    # 処理周期の設定
    r = rospy.Rate(CYCLES)

    print("[sampleDB] start")


    # ctl +　Cで終了しない限りwhileループで処理し続ける
    while not rospy.is_shutdown():
        # メイン処理
        #sampleDB.main()
        #
        r.sleep()

if __name__ == '__main__':
    try:
        sampleDB_py()

    except rospy.ROSInterruptException:
        print("[sampleDB] end")
