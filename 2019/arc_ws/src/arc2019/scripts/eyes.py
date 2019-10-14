#!/usr/bin/python
# -*- coding: utf-8 -*-
# license removed for brevity

# pythonでROSのソフトウェアを記述するときにimportするモジュール
import rospy

# 自分で定義したmessageファイルから生成されたモジュール
from arc2019.msg import brain
from arc2019.msg import eye

# 定数などの定義ファイルimport
from params import MODE, TARGET, CAMERA, INVALID_POS

class Eyes(object):
    """
    2眼カメラから対象物を検出するクラス
    （通信部分のみ実装）
    """
#--------------------
# コンストラクタ
    def __init__(self, id):
        """
        コンストラクタ
        """
        self.myid = id
        # 受信作成
        self.sub_brain  = rospy.Subscriber('brain', brain, self.brainCallback, queue_size=1)
        # 送信作成
        self.pub_eye = rospy.Publisher('eye', eye, queue_size=100)
        # messageのインスタンスを作る
        self.msg_eye = eye()
        #デフォルト
        self.clearMsg()

    def clearMsg(self):
        """
        publishするメッセージのクリア
        """
        #for all
        self.msg_eye.camera_id = self.myid
        self.msg_eye.target_find = False
        self.msg_eye.target_x = INVALID_POS
        self.msg_eye.target_y = INVALID_POS
        self.msg_eye.target_z = INVALID_POS

#--------------------
# 受信コールバック
    def brainCallback(self, brain_msg):
        """
        脳の受信コールバック
        """
        pass

#--------------------
    def update(self, x=INVALID_POS, y=INVALID_POS, z=INVALID_POS):
        """
        publishするデータの更新
        """
        self.clearMsg()
        self.msg_eye.target_x = x
        self.msg_eye.target_y = y
        self.msg_eye.target_z = z

        if x>INVALID_POS and y>INVALID_POS and z>INVALID_POS:
            self.msg_eye.target_find = True
        else:
            self.msg_eye.target_find = False

    def main(self):
        """
        メインシーケンス
        """
        self.pub_eye.publish(self.msg_eye)
