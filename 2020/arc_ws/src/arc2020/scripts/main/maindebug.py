#!/usr/bin/python
# -*- coding: utf-8 -*-
# license removed for brevity

# pythonでROSのソフトウェアを記述するときにimportするモジュール
import rospy

# gpio制御用
import pigpio
import time

# 自分で定義したmessageファイルから生成されたモジュール
#受信
from arc2020.msg import webdebug
from arc2020.msg import main

#送信
from arc2020.msg import armdebugclient
from arc2020.msg import foot
from arc2020.msg import maindebug


# 定数などの定義ファイルimport


CYCLES = 60 

class MainDebug(object):
    """
    全体の動作を制御するクラス
    """
#--------------------
# コンストラクタ
    def __init__(self):
        """
        コンストラクタ
        """
        # 受信作成
        # webapp⇒maindebug
        self.sub_webdebug  = rospy.Subscriber('webdebug', webdebug, self.webdebugCallback, queue_size=1)
        
        # main⇒maindebug
        self.sub_main  = rospy.Subscriber('main', main, self.mainCallback, queue_size=1)
        
        # foot⇒maindebug
        self.sub_foot  = rospy.Subscriber('foot_sensor', foot, self.footCallback, queue_size=1)
        
        
        # 送信作成
        # maindebug⇒arm
        self.pub_armdebugclient = rospy.Publisher('armdebugclient', armdebugclient, queue_size=100)
        self.msg_armdebugclient = armdebugclient()
        
        # maindebug⇒foot
        self.pub_foot = rospy.Publisher('foot_debug', foot, queue_size=100)
        self.msg_foot = foot()
        
        # maindebug⇒webapp
        self.pub_maindebug = rospy.Publisher('maindebug', maindebug, queue_size=100)
        self.msg_maindebug = maindebug()

        # 送信メッセージ初期化
        self.clearMsg()
        
#--------------------
# 送信メッセージの初期化
    def clearMsg(self):
        """
        publishするメッセージのクリア
        """
        # maindebug⇒arm
        self.msg_armdebugclient.drill_req = 0
        self.msg_armdebugclient.drill_height_value = 0
        self.msg_armdebugclient.drill_width_value = 0
        
        # maindebug⇒foot
        self.msg_foot.motor_r = 0
        self.msg_foot.motor_l = 0
        
        # maindebug⇒webapp
        self.msg_maindebug.volt = 0
        self.msg_maindebug.cur = 0
        for i in range(2):
            self.msg_maindebug.sonor.append(0)
        for i in range(8):
            self.msg_maindebug.line.append(0)
        for i in range(3):
            self.msg_maindebug.accel.append(0)
        for i in range(3):
            self.msg_maindebug.gyro.append(0)
        for i in range(3):
            self.msg_maindebug.mag.append(0)

#--------------------
# 受信コールバック
    def webdebugCallback(self, webdebug_msg):
        """
        クライアントの受信コールバック
        """
        self.msg_armdebugclient.drill_req = webdebug_msg.move_stop
        self.msg_armdebugclient.drill_height_value = webdebug_msg.arm_height_value
        self.msg_armdebugclient.drill_width_value = webdebug_msg.arm_width_value
        
        self.msg_foot.motor_r = webdebug_msg.motor_r
        self.msg_foot.motor_l = webdebug_msg.motor_l
        
#--------------------
# 受信コールバック
    def mainCallback(self, main_msg):
        """
        クライアントの受信コールバック
        """
        self.msg_maindebug.volt = main_msg.volt
        self.msg_maindebug.cur = main_msg.cur

#--------------------
# 受信コールバック
    def footCallback(self, foot_msg):
        """
        クライアントの受信コールバック
        """
        for i in range(2):
            self.msg_maindebug.sonor[i] = foot_msg.sonor[i]
        for i in range(8):
            self.msg_maindebug.line[i] = foot_msg.line[i]
        for i in range(3):
            self.msg_maindebug.accel[i] = foot_msg.accel[i]
        for i in range(3):
            self.msg_maindebug.gyro[i] = foot_msg.gyro[i]
        for i in range(3):
            self.msg_maindebug.mag[i] = foot_msg.mag[i]

#--------------------
# メイン関数
    def main(self):
        
        # メッセージを発行する
        self.pub_armdebugclient.publish(self.msg_armdebugclient)
        self.pub_foot.publish(self.msg_foot)
        self.pub_maindebug.publish(self.msg_maindebug)
        
#--------------------
def maindebug_py():
    # 初期化宣言 : このソフトウェアは"maindebug_py_node"という名前
    rospy.init_node('maindebug_py_node', anonymous=True)
    # インスタンスの作成 
    maindebug = MainDebug()
    # 処理周期の設定
    r = rospy.Rate(CYCLES)

    print("[maindebug] start")
    # ctl +　Cで終了しない限りwhileループで処理し続ける
    while not rospy.is_shutdown():
        # メイン処理
        maindebug.main()
        #
        r.sleep()

if __name__ == '__main__':
    try:
        maindebug_py()

    except rospy.ROSInterruptException:
        print("[maindebug] end")
