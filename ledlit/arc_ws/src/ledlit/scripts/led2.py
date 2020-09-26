#!/usr/bin/python
# -*- coding: utf-8 -*-
# license removed for brevity

# pythonでROSのソフトウェアを記述するときにimportするモジュール
import rospy

# gpio制御用
import pigpio

# 自分で定義したmessageファイルから生成されたモジュール
from ledlit.msg import brain

PIN_C = 19
PINS = [PIN_C]
CYCLES = 60 #処理周波数
# 定数などの定義ファイルimport


# class Led2 定義
class Led2(object):
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
        self.sub_client  = rospy.Subscriber('brain', brain, self.brainCallback, queue_size=1)
        # messageのインスタンスを作る
        self.msg_brain = brain()

        #GPIOの初期設定
        self.pi = pigpio.pi()
        for pin in PINS :
            self.pi.set_mode(pin, pigpio.OUTPUT)
            self.pi.set_PWM_frequency(pin, 50)
            self.pi.set_PWM_range(pin, 100)
        

#--------------------
# 受信コールバック
    def brainCallback(self, brain_msg):
        """
        brainの受信コールバック
        """
        #print("led a" + str(brain_msg.led_a_value)) 
        self.pi.set_PWM_dutycycle(PIN_C,brain_msg.led_c_value)
#--------------------
#    def main(self,mm):
        # 特に処理なし

def led2_py():
    # 初期化宣言 : このソフトウェアは"led2_py_node"という名前
    rospy.init_node('led2_py_node', anonymous=True)
    # インスタンスの作成 
    led2 = Led2()
    # 処理周期の設定
    r = rospy.Rate(CYCLES)

    print("[led2] start")
    # ctl +　Cで終了しない限りwhileループで処理し続ける
    while not rospy.is_shutdown():
        # メイン処理
        #led.main()
        #
        r.sleep()

if __name__ == '__main__':
    try:
        led2_py()

    except rospy.ROSInterruptException:
        print("[led2] end")
