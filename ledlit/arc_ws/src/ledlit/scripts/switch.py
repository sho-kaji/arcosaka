#!/usr/bin/python
# -*- coding: utf-8 -*-
# license removed for brevity

# pythonでROSのソフトウェアを記述するときにimportするモジュール
import rospy

# gpio制御用
import pigpio

# 自分で定義したmessageファイルから生成されたモジュール
from ledlit.msg import switch

PIN = 20

CYCLES = 60 #処理周波数
# 定数などの定義ファイルimport


# class Led 定義
class Switch(object):
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
        self.pub_switch = rospy.Publisher('switch', switch, queue_size=100)
        # messageのインスタンスを作る
        self.msg_switch = switch()

        #GPIOの初期設定
        self.pi = pigpio.pi()
        self.pi.set_mode(PIN, pigpio.INPUT)

        # 送信メッセージ初期化
        self.clearMsg()
#--------------------
# 送信メッセージの初期化
    def clearMsg(self):
        """
        publishするメッセージのクリア
        """
        self.msg_switch.switch_status = 0
#--------------------
    def main(self):
        if self.pi.read(PIN) :
            self.msg_switch.switch_status = 0
        else :
            self.msg_switch.switch_status = 1
        # メッセージを発行する
        self.pub_switch.publish(self.msg_switch)

def switch_py():
    # 初期化宣言 : このソフトウェアは"switch_py_node"という名前
    rospy.init_node('switch_py_node', anonymous=True)
    # インスタンスの作成 
    switch = Switch()
    # 処理周期の設定
    r = rospy.Rate(CYCLES)

    print("[switch] start")
    # ctl +　Cで終了しない限りwhileループで処理し続ける
    while not rospy.is_shutdown():
        # メイン処理
        switch.main()
        #
        r.sleep()

if __name__ == '__main__':
    try:
        switch_py()

    except rospy.ROSInterruptException:
        print("[switch] end")
