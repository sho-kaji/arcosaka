#!/usr/bin/python
# -*- coding: utf-8 -*-
# license removed for brevity

# pythonでROSのソフトウェアを記述するときにimportするモジュール
import rospy

# gpio制御用
#import pigpio

# 自作ライブラリ
#from Vec3D import Vec3D
#from Transform3D import Transform3D

# 自分で定義したmessageファイルから生成されたモジュール
from ledlit.msg import client
from ledlit.msg import brain

# 定数などの定義ファイルimport
#from params import MODE, TARGET, CAMERA, DIRECTION

CYCLES = 60 

class Brain(object):
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
        self.sub_client  = rospy.Subscriber('client2', client, self.clientCallback, queue_size=1)
        # 送信作成
        self.pub_brain = rospy.Publisher('brain', brain, queue_size=100)
        # messageのインスタンスを作る
        self.msg_brain = brain()
        # 送信メッセージ初期化
        self.clearMsg()
#--------------------
# 送信メッセージの初期化
    def clearMsg(self):
        """
        publishするメッセージのクリア
        """
        #for all
        self.msg_brain.led_a_value  = 0
        self.msg_brain.led_b_value  = 0
#--------------------
# 受信コールバック
    def clientCallback(self, client_msg):
        """
        クライアントの受信コールバック
        """
        #print(client_msg.led_a_value)
        if client_msg.mode :
            self.msg_brain.led_a_value = client_msg.led_a_value
            self.msg_brain.led_b_value = client_msg.led_b_value
        else : 
            self.msg_brain.led_a_value = 0 
            self.msg_brain.led_b_value = 0 
#--------------------
    def main(self):
        # メッセージを発行する
        self.pub_brain.publish(self.msg_brain)


def brain_py():
    # 初期化宣言 : このソフトウェアは"brain_py_node"という名前
    rospy.init_node('brain_py_node', anonymous=True)
    # インスタンスの作成 
    brain = Brain()
    # 処理周期の設定
    r = rospy.Rate(CYCLES)

    print("[brain] start")
    # ctl +　Cで終了しない限りwhileループで処理し続ける
    while not rospy.is_shutdown():
        # メイン処理
        brain.main()
        #
        r.sleep()

if __name__ == '__main__':
    try:
        brain_py()

    except rospy.ROSInterruptException:
        print("[brain] end")
