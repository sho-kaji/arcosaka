#!/usr/bin/python
# -*- coding: utf-8 -*-
# license removed for brevity

# pythonでROSのソフトウェアを記述するときにimportするモジュール
import rospy

# gpio制御用
import pigpio
import time

# モーター制御用
import mortor

# 自分で定義したmessageファイルから生成されたモジュール
from arc2020.msg import armdebugclient
from arc2020.msg import servo
from arc2020.msg import step

from arc2020.msg import armdebug

# 定数などの定義ファイルimport
from arm_consts import *

CYCLES = 60 

class ArmDebug(object):
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
        self.sub_armdebugclient  = rospy.Subscriber('armdebugclient', armdebugclient, self.clientCallback, queue_size=1)
        self.sub_servo  = rospy.Subscriber('servo', servo, self.servoCallback, queue_size=1)
        self.sub_step  = rospy.Subscriber('step', step, self.stepCallback, queue_size=1)
        
        # 送信作成
        self.pub_armdebug = rospy.Publisher('armdebug', armdebug, queue_size=100)
        
        # messageのインスタンスを作る
        self.msg_armdebug = armdebug()
        
        # 送信メッセージ初期化
        self.clearMsg()
        
        self.servostatus = 0
        self.stepstatus = 0
        
#--------------------
# 送信メッセージの初期化
    def clearMsg(self):
        """
        publishするメッセージのクリア
        """
        #for all
        self.msg_armdebug.drill_req = 0
        self.msg_armdebug.armdrillstatus = 0
        self.msg_armdebug.drill_height_value = 0
        self.msg_armdebug.drill_width_value = 0
        self.msg_armdebug.armretorgsw = 0
        
#--------------------
# 受信コールバック
    def clientCallback(self, armdebugclient_msg):
        """
        クライアントの受信コールバック
        """
        print("drill_req = %d" % (armdebugclient_msg.drill_req))
        # メッセージ受信
        
        if armdebugclient_msg.drill_req :
            print("width_value = %d" % (armdebugclient_msg.drill_width_value))
            print("height_value = %d" % (armdebugclient_msg.drill_height_value))
            
            self.msg_armdebug.drill_req = armdebugclient_msg.drill_req
            self.msg_armdebug.drill_width_value = armdebugclient_msg.drill_width_value
            self.msg_armdebug.drill_height_value = armdebugclient_msg.drill_height_value
        
#--------------------
# 受信コールバック
    def servoCallback(self, servo_msg):
        """
        クライアントの受信コールバック
        """
        self.servostatus = servo_msg.servostatus
        
#--------------------
# 受信コールバック
    def stepCallback(self, step_msg):
        """
        クライアントの受信コールバック
        """
        self.msg_armdebug.armretorgsw = step_msg.retorgsw
        self.stepstatus = step_msg.stepstatus

#--------------------
# メイン関数
    def main(self):
        if self.servostatus == 1 or self.stepstatus == 1 :
            self.msg_armdebug.armdrillstatus = 1
        else :
            self.msg_armdebug.armdrillstatus = 0
        
        # メッセージを発行する
        self.pub_armdebug.publish(self.msg_armdebug)
        
#--------------------
def armdebug_py():
    # 初期化宣言 : このソフトウェアは"armdebug_py_node"という名前
    rospy.init_node('armdebug_py_node', anonymous=True)
    # インスタンスの作成 
    armdebug = ArmDebug()
    # 処理周期の設定
    r = rospy.Rate(CYCLES)

    print("[armdebug] start")
    # ctl +　Cで終了しない限りwhileループで処理し続ける
    while not rospy.is_shutdown():
        # メイン処理
        armdebug.main()
        #
        r.sleep()

if __name__ == '__main__':
    try:
        armdebug_py()

    except rospy.ROSInterruptException:
        print("[armdebug] end")
