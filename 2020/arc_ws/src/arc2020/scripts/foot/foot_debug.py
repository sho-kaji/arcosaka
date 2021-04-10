#!/usr/bin/python
# -*- coding: utf-8 -*-
# license removed for brevity

# pythonでROSのソフトウェアを記述するときにimportするモジュール
import rospy

# gpio制御用
import pigpio

# 自分で定義したmessageファイルから生成されたモジュール
from arc2020.msg import foot


from dc_motor import DCMotor
#import GPIOPIN
from GPIO import GPIOPIN
# class Led 定義

CYCLES = 60

class FootDebug(object):
    """
    スイッチの動作を制御するクラス
    """
#--------------------
# コンストラクタ
    def __init__(self):
        """
        コンストラクタ
        """
        # 送信作成
        #self.pub_mileage = rospy.Publisher('mileage', mileage, queue_size=100)
        # messageのインスタンスを作る
        self.msg_foot = foot()
        self.motor_r =  DCMotor(GPIOPIN.DC_MOTOR_A1.value,GPIOPIN.DC_MOTOR_A2.value)
        self.motor_l =  DCMotor(GPIOPIN.DC_MOTOR_B1.value,GPIOPIN.DC_MOTOR_B2.value)
        rospy.Subscriber('foot_debug', foot, self.callback, queue_size=1)
        
        # 送信メッセージ初期化
        #self.clearMsg()
    def callback(self,msg):
        self.motor_r.changeDuty(msg.motor_r)
        self.motor_l.changeDuty(msg.motor_l)
#--------------------
    def main(self):
        i = 1
        # メッセージを発行する
        #print(self.msg_mileage.mileage)
        #self.pub_mileage.publish(self.msg_mileage)


def foot_debug_py():
    # 初期化宣言 : このソフトウェアは"mileage_py_node"という名前
    rospy.init_node('foot_debug_py_node', anonymous=True)
    # インスタンスの作成 
    foot = FootDebug()
    # 処理周期の設定
    r = rospy.Rate(CYCLES)

    print("[foot_debug] start")
    # ctl +　Cで終了しない限りwhileループで処理し続ける
    while not rospy.is_shutdown():
        # メイン処理
        foot.main()
        #
        r.sleep()

if __name__ == '__main__':
    try:
        foot_debug_py()

    except rospy.ROSInterruptException:
        print("[foot_debug] end")
