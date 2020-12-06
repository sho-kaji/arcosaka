#!/usr/bin/python
# -*- coding: utf-8 -*-
# license removed for brevity

# pythonでROSのソフトウェアを記述するときにimportするモジュール
import rospy

# gpio制御用
import pigpio

# 自分で定義したmessageファイルから生成されたモジュール
from arc2020.msg import main

PIN_A = 17
PIN_B = 18
PINS = [PIN_A,PIN_B]
CYCLES = 60 #処理周波数
# 定数などの定義ファイルimport


# class Led 定義
class Led(object):
    """
    LEDの動作を制御するクラス
    """
#--------------------
# コンストラクタ
    def __init__(self):
        """
        コンストラクタ
        """
        self.cyclecount = 0
        # 受信作成
        self.sub_client  = rospy.Subscriber('main', main, self.mainCallback, queue_size=1)
        # messageのインスタンスを作る
        self.msg_main = main()

        #GPIOの初期設定
        self.pi = pigpio.pi()
        for pin in PINS :
            self.pi.set_mode(pin, pigpio.OUTPUT)
            self.pi.set_PWM_frequency(pin, 50)
            self.pi.set_PWM_range(pin, 100)
        

#--------------------
# 受信コールバック
    def mainCallback(self, main_msg):
        """
        mainの受信コールバック
        """
        #print("led a" + str(main_msg.led_a_value)) 
        self.pi.set_PWM_dutycycle(PIN_A,main_msg.led_a_value)
        self.pi.set_PWM_dutycycle(PIN_B,main_msg.led_b_value)
#--------------------
#    def main(self,mm):
        # 特に処理なし

def led_py():
    # 初期化宣言 : このソフトウェアは"led_py_node"という名前
    rospy.init_node('led_py_node', anonymous=True)
    # インスタンスの作成 
    led = Led()
    # 処理周期の設定
    r = rospy.Rate(CYCLES)

    print("[led] start")
    # ctl +　Cで終了しない限りwhileループで処理し続ける
    while not rospy.is_shutdown():
        # メイン処理
        #led.main()
        #
        r.sleep()

if __name__ == '__main__':
    try:
        led_py()

    except rospy.ROSInterruptException:
        print("[led] end")
