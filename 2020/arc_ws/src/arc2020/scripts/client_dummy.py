#!/usr/bin/python
# -*- coding: utf-8 -*-
# license removed for brevity

# pythonでROSのソフトウェアを記述するときにimportするモジュール
import rospy

# 自分で定義したmessageファイルから生成されたモジュール
from arc2020.msg import brain
from arc2020.msg import client

# 定数などの定義ファイルimport
from client_consts import  \
    DEFAULT_MODE, DEFAULT_TARGET, CYCLES
from params import MODE, TARGET

class Client(object):
    """
    モード、ターゲットを決定するクラス
    マニュアルモード時に直接操作するクラス
    """
#--------------------
# コンストラクタ
    def __init__(self):
        """
        コンストラクタ
        """
        self.cyclecount = 0
        # 受信作成
        self.sub_brain  = rospy.Subscriber('brain', brain, self.brainCallback, queue_size=1)
        # 送信作成
        self.pub_client = rospy.Publisher('client', client, queue_size=100)
        # messageのインスタンスを作る
        self.msg_client = client()
        #デフォルト
        self.mode = DEFAULT_MODE
        self.target = DEFAULT_TARGET

        self.clearMsg()

    def clearMsg(self):
        """
        publishするメッセージのクリア
        """
        #for all
        self.msg_client.mode = self.mode
        self.msg_client.target = self.target

#--------------------
# 受信コールバック
    def brainCallback(self, brain_msg):
        """
        脳の受信コールバック
        """
        if brain_msg.mode_id == MODE.INIT:
            self.pub_client.publish(self.msg_client)
            print("[client_dummy] pulish")

#--------------------

    def main(self):
        """
        メインシーケンス
        """
        pass

def client_py():
    # 初期化宣言 : このソフトウェアは"dummyclient_py_node"という名前
    rospy.init_node('dummyclient_py_node', anonymous=True)
    # インスタンスの作成 
    client = Client()
    # 処理周期の設定
    r = rospy.Rate(CYCLES)

    print("[client_dummy] start")
    # ctl +　Cで終了しない限りwhileループで処理し続ける
    while not rospy.is_shutdown():
        # メイン処理
        client.main()
        #
        r.sleep()

if __name__ == '__main__':
    try:
        client_py()

    except rospy.ROSInterruptException:
        print("[client_dummy] end")
