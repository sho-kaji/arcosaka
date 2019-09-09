#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
ボディ
"""


import pigpio
import rospy

from arc2019.msg import body
from params import Mode, TARGET

from body_consts import \
                DEBUG_BODY,\
                PORT_LID,\
                PORT_SPRAY, \
                PORT_BLADE_A, PORT_BLADE_B,\
                PORT_PWOFFSW \

from brain_consts import PUBLISH_RATE

class BodyClass():
    """
    ボディを動かすためのクラス
    """

    def __init__(self):

        # initialize gpio
        self.pic = pigpio.pi()
        self.pic.set_mode(PORT_LID, pigpio.OUTPUT)
        self.pic.set_mode(PORT_SPRAY, pigpio.OUTPUT)
        self.pic.set_mode(PORT_BLADE_A, pigpio.OUTPUT)
        self.pic.set_mode(PORT_BLADE_B, pigpio.OUTPUT)
        self.pic.set_mode(PORT_PWOFFSW, pigpio.INPUT)

        # パブリッシャーの準備
        self.pub_body = rospy.Publisher('body', body, queue_size=100)
        # messageのインスタンスを作る
        self.msg_body = body()
        # index
        self.frame_id = 0
        # モード前回値
        self.mode_old = Mode.UNKNOWN

    def callback(self, bodymes):
        """
        メッセージを受信したときに呼び出し
        """

        print('frame_id = %d ' % bodymes.frame_id)

        #モード変更確認
        self.grubMotion(bodymes.mode)

        #区切り
        print("==============================")

    def modeChange(self, mode):
        """
        モード変更処理
        """

        if mode != self.mode_old:

            #モード変更時初期化

            #モーター停止

            self.mode_old = mode

        else:
            pass

        if self.mode_old > -1:
            print("mode = %s" % Mode(self.mode_old).name)
        else:
            print("mode = %s" % "UNKNOWN")

    def grubMotion(self, grub):
        """
        掴む/放す
        """


    def clear_msg(self):
        """
        メッセージ初期化
        """
        self.msg_body.is_body_move = False
        self.msg_body.is_pwoffsw = False
        
    def publishData(self):
        """
        データ送信
        """
        # clear
        self.clear_msg()
        self.msg_body.frame_id = self.frame_id
        # publishする関数
        self.pub_body.publish(self.msg_body)
        self.frame_id += 1


def printDebug(message):
    """
    デバッグメッセージを表示
    """
    if DEBUG_BODY is True:
        print(message)
    else:
        pass

def body_py():
    """
    ボディのメイン
    """

    bodyc = BodyClass()
    r = rospy.Rate(PUBLISH_RATE)
    rospy.init_node('body_py_node', anonymous=True)
    rospy.Subscriber('body', body, bodyc.callback, queue_size=1)
    printDebug("start_body")
    # ctl +　Cで終了しない限りwhileループでpublishし続ける
    while not rospy.is_shutdown():
        # publishする関数
        bodyc.publishData()
        #
        r.sleep()

if __name__ == '__main__':
    body_py()
