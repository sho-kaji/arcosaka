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
from arc2020.msg import armdebug

# 定数などの定義ファイルimport
from arm_consts import *

RET_ORGSW = 15

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
        self.cyclecount = 0
        # 受信作成
        self.sub_armdebugclient  = rospy.Subscriber('client2', armdebugclient, self.clientCallback, queue_size=1)
        # 送信作成
        self.pub_armdebug = rospy.Publisher('armdebug', armdebug, queue_size=100)
        # messageのインスタンスを作る
        self.msg_armdebug = armdebug()
        # 送信メッセージ初期化
        self.clearMsg()
        
        # メッセージ受信用変数
        #self.drill_req = 0  # 受信値B
        
        # MortorClass
        self.stmc = mortor.StepMortorClass(DEBUG_ARM, (PORT_HANDV_A, PORT_HANDV_B), (LIM_HANDV_MIN, LIM_HANDV_MAX))
        self.svmc = mortor.ServoMortorClass(DEBUG_ARM)
        
        self.pi = pigpio.pi()
        self.pi.set_mode(RET_ORGSW, pigpio.INPUT)
        
#--------------------


# 送信メッセージの初期化
    def clearMsg(self):
        """
        publishするメッセージのクリア
        """
        #for all
        self.msg_armdebug.armdrillstatus = 0
        self.msg_armdebug.armretorgstatus = 0
        self.msg_armdebug.armretorgsw = 0
#--------------------


# 受信コールバック
    def clientCallback(self, armdebugclient_msg):
        """
        クライアントの受信コールバック
        """
        print("retorgstatus = %d" % (self.msg_armdebug.armretorgstatus))
        print("drill_req = %d" % (armdebugclient_msg.drill_req))
        # メッセージ受信
        
        if armdebugclient_msg.drill_req :
            if self.msg_armdebug.armretorgstatus == 0 : # 原点復帰未実施
                self.msg_armdebug.armdrillstatus = 1 # アーム動作中
                self.svmc.move_servo(CHANNEL_HAND, RELEASE_HAND)
                self.stmc.move_posinit_step()
                self.msg_armdebug.armdrillstatus = 0 # アーム停止中
                self.msg_armdebug.armretorgstatus = 1 # 原点復帰実施済み
            
            if self.msg_armdebug.armretorgstatus == 1 : # 原点復帰実施済み
                # 処理 y軸移動⇒z軸降下⇒z軸上昇
                self.msg_armdebug.armdrillstatus = 1
                
                print("width_value = %d" % (armdebugclient_msg.drill_width_value))
                print("height_value = %d" % (armdebugclient_msg.drill_height_value))
                
                
                handy = armdebugclient_msg.drill_width_value
                self.stmc.move_step(handy)  # y軸移動
                time.sleep(1)
                
                
                handz = armdebugclient_msg.drill_height_value
                self.svmc.move_servo(CHANNEL_HAND, handz)  # z軸移動
                time.sleep(1)
                
                
                # 送信用メッセージ更新
                self.msg_armdebug.armdrillstatus = 0
        
        self.pi.set_mode(RET_ORGSW, pigpio.INPUT)
        if self.pi.read(RET_ORGSW) :
            self.msg_armdebug.armretorgsw = 0
        else :
            self.msg_armdebug.armretorgsw = 1
#--------------------


# メイン関数
    def main(self):
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
