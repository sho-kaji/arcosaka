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
from arc2020.msg import arm

#送信
from arc2020.msg import servo

# 定数などの定義ファイルimport
from arm_consts import *

CYCLES = 60 

class Servo(object):
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
        self.sub_arm  = rospy.Subscriber('arm', arm, self.armCallback, queue_size=1)
        
        # 送信作成
        self.pub_servo = rospy.Publisher('servo', servo, queue_size=100)
        # messageのインスタンスを作る
        self.msg_servo = servo()
        
        # 送信メッセージ初期化
        self.clearMsg()
        
        # MortorClass
        self.svmc = mortor.ServoMortorClass(DEBUG_ARM)
        
#--------------------
# 送信メッセージの初期化
    def clearMsg(self):
        """
        publishするメッセージのクリア
        """
        self.msg_servo.servostatus = 0
        
#--------------------
# 受信コールバック
    def armCallback(self, arm_msg):
        """
        クライアントの受信コールバック
        """
        # メッセージ受信
        
        if arm_msg.drill_req :
            # 処理 y軸移動⇒z軸降下⇒z軸上昇
            
            handz = arm_msg.drill_height_value
            if handz < 30 :
                handz = 30
            elif handz > 75 :
                handz = 75
            
            self.msg_servo.servostatus = 1
            self.svmc.move_servo(CHANNEL_HAND, handz)  # z軸移動
            time.sleep(1)
            self.msg_servo.servostatus = 0
            
#--------------------
# メイン関数
    def main(self):
        # メッセージを発行する
        self.pub_servo.publish(self.msg_servo)
        
#--------------------
def servo_py():
    # 初期化宣言 : このソフトウェアは"servo_py_node"という名前
    rospy.init_node('servo_py_node', anonymous=True)
    # インスタンスの作成 
    servo = Servo()
    # 処理周期の設定
    r = rospy.Rate(CYCLES)

    print("[servo] start")
    # ctl +　Cで終了しない限りwhileループで処理し続ける
    while not rospy.is_shutdown():
        # メイン処理
        servo.main()
        #
        r.sleep()

if __name__ == '__main__':
    try:
        servo_py()

    except rospy.ROSInterruptException:
        print("[servo] end")
