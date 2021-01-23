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
from arc2020.msg import client
from arc2020.msg import main
from arc2020.msg import emg
from arc2020.msg import volcurmeas

# 定数などの定義ファイルimport
#from params import MODE, TARGET, CAMERA, DIRECTION

CYCLES = 60 

class Main(object):
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
#        self.sub_client  = rospy.Subscriber('client2', client, self.clientCallback, queue_size=1)
        self.sub_emg  = rospy.Subscriber('emg', emg, self.emgCallback, queue_size=1)
        self.sub_volcurmeas  = rospy.Subscriber('volcurmeas', volcurmeas, self.volcurmeasCallback, queue_size=1)
        # 送信作成
        self.pub_main = rospy.Publisher('main', main, queue_size=100)
        # messageのインスタンスを作る
        self.msg_main = main()
        # 送信メッセージ初期化
        self.clearMsg()
#--------------------
# 送信メッセージの初期化
    def clearMsg(self):
        """
        publishするメッセージのクリア
        """
        #for all
#        self.msg_main.led_a_value  = 0
#        self.msg_main.led_b_value  = 0
#        self.msg_main.led_c_value  = 0
#--------------------
# 受信コールバック
#    def clientCallback(self, client_msg):
#        """
#        クライアントの受信コールバック
#        """
#        #print(client_msg.led_a_value)
#        if client_msg.mode :
#            self.msg_main.led_a_value = client_msg.led_a_value
#            self.msg_main.led_b_value = client_msg.led_b_value
#            self.msg_main.led_c_value = client_msg.led_c_value
#        else : 
#            self.msg_main.led_a_value = 0 
#            self.msg_main.led_b_value = 0 
#            self.msg_main.led_c_value = 0
    def emgCallback(self, emg_msg):
        """
        クライアントの受信コールバック
        """
        self.msg_main.emg = emg_msg.emg
    def volcurmeasCallback(self, volcurmeas_msg):
        """
        クライアントの受信コールバック
        """
        self.msg_main.volt = volcurmeas_msg.volt
        self.msg_main.cur = volcurmeas_msg.cur
#--------------------
    def main(self):
        # メッセージを発行する
        self.pub_main.publish(self.msg_main)


def main_py():
    # 初期化宣言 : このソフトウェアは"main_py_node"という名前
    rospy.init_node('main_py_node', anonymous=True)
    # インスタンスの作成 
    main = Main()
    # 処理周期の設定
    r = rospy.Rate(CYCLES)

    print("[main] start")
    # ctl +　Cで終了しない限りwhileループで処理し続ける
    while not rospy.is_shutdown():
        # メイン処理
        main.main()
        #
        r.sleep()

if __name__ == '__main__':
    try:
        main_py()

    except rospy.ROSInterruptException:
        print("[main] end")
