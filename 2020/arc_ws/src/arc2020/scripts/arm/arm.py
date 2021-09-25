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
#受信
from arc2020.msg import main
from arc2020.msg import servo
from arc2020.msg import step

#送信
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
        # 受信作成
        self.sub_main  = rospy.Subscriber('main', main, self.mainCallback, queue_size=1)
        self.sub_servo  = rospy.Subscriber('servo', servo, self.servoCallback, queue_size=1)
        self.sub_step  = rospy.Subscriber('step', step, self.stepCallback, queue_size=1)
        
        # 送信作成
        self.pub_arm = rospy.Publisher('arm', arm, queue_size=100)
        
        # messageのインスタンスを作る
        self.msg_arm = arm()
        
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
        self.msg_arm.drill_req = 0
        self.msg_arm.armdrillstatus = 0
        self.msg_arm.drill_height_value = 0
        self.msg_arm.drill_width_value = 0
        self.msg_arm.armretorgsw = 0
        
#--------------------
# 受信コールバック
    def mainCallback(self, main_msg):
        """
        クライアントの受信コールバック
        """
        # メッセージ受信
        
        self.msg_arm.drill_req = main_msg.drill_req
        self.msg_arm.drill_width_value = main_msg.drill_width_value
        self.msg_arm.drill_height_value = main_msg.drill_height_value
        
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
        self.msg_arm.armretorgsw = step_msg.retorgsw
        self.stepstatus = step_msg.stepstatus
        

#--------------------
# メイン関数
    def main(self):
        if self.servostatus == 1 or self.stepstatus == 1 :
            self.msg_arm.armdrillstatus = 1
        else :
            self.msg_arm.armdrillstatus = 0
        
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
