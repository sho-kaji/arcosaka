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
from arc2020.msg import brain
from arc2020.msg import arm

# 定数などの定義ファイルimport
from arm_consts import *

CYCLES = 60 

class Arm(object):
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
        self.sub_brain  = rospy.Subscriber('client2', brain, self.clientCallback, queue_size=1)
        # 送信作成
        self.pub_arm = rospy.Publisher('arm', arm, queue_size=100)
        # messageのインスタンスを作る
        self.msg_arm = arm()
        # 送信メッセージ初期化
        self.clearMsg()
        
        # メッセージ受信用変数
        #self.drill_req = 0  # 受信値B
        
        # MortorClass
        self.stmc = mortor.StepMortorClass(DEBUG_ARM, (PORT_HANDV_A, PORT_HANDV_B), (LIM_HANDV_MIN, LIM_HANDV_MAX))
        self.svmc = mortor.ServoMortorClass(DEBUG_ARM)
#--------------------


# 送信メッセージの初期化
    def clearMsg(self):
        """
        publishするメッセージのクリア
        """
        #for all
        self.msg_arm.armdrillstatus = 0
        self.msg_arm.armretorgstatus = 0
#--------------------


# アーム原点復帰
    def posinit(self):
        """
        位置初期化
        """
        self.svmc.move_servo(CHANNEL_HAND, RELEASE_HAND)
        self.stmc.move_posinit_step()
#--------------------


# アーム動作
    def move_arm(self):
        """
        y軸移動⇒z軸下降⇒z軸上昇
        """
        
        # y軸移動
        handy = brain_msg.drill_width_value
        self.stmc.move_step(handy)  # y軸移動
        
        # z軸移動
        self.svmc.move_servo(CHANNEL_HAND, RELEASE_HAND) # z軸上昇
        time.sleep(2)
        self.svmc.move_servo(CHANNEL_HAND, CATCH_HAND) # z軸降下
        time.sleep(2)
        self.svmc.move_servo(CHANNEL_HAND, RELEASE_HAND) # z軸上昇
        time.sleep(2)
#--------------------


# 受信コールバック
    def clientCallback(self, brain_msg):
        """
        クライアントの受信コールバック
        """
        if brain_msg.retorg_req == 1 : # アーム原点復帰要求有無
            if self.msg_arm.armretorgstatus == 0 : # 原点復帰実施状態判定
                self.msg_arm.armdrillstatus = 1 # アーム動作中
                self.posinit()
                self.msg_arm.armdrillstatus = 0 # アーム停止中
                self.msg_arm.armretorgstatus = 1 # 原点復帰実施済み
        
        if brain_msg.drill_req == 1 and self.msg_arm.armretorgstatus == 1 : # アーム動作指示要求有無
            self.msg_arm.armdrillstatus = 1 # アーム動作中
            self.move_arm() # アーム動作
            self.msg_arm.armdrillstatus = 0 # アーム停止中
#--------------------


# メイン関数
    def main(self):
        # メッセージを発行する
        self.pub_arm.publish(self.msg_arm)
#--------------------


def arm_py():
    # 初期化宣言 : このソフトウェアは"arm_py_node"という名前
    rospy.init_node('arm_py_node', anonymous=True)
    # インスタンスの作成 
    arm = Arm()
    # 処理周期の設定
    r = rospy.Rate(CYCLES)

    print("[arm] start")
    # ctl +　Cで終了しない限りwhileループで処理し続ける
    while not rospy.is_shutdown():
        # メイン処理
        arm.main()
        #
        r.sleep()

if __name__ == '__main__':
    try:
        arm_py()

    except rospy.ROSInterruptException:
        print("[arm] end")
