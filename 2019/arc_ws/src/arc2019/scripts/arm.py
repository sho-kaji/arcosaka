#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
アーム
"""


import pigpio
import rospy

from arc2019.msg import arm
from params import Mode, TARGET

from arm_consts import \
                DEBUG_ARM, \
                LIM_BASE_L, LIM_BASE_R, \
                LIM_ELBOW_B, LIM_ELBOW_F, \
                LIM_SHOULD_B, LIM_SHOULD_F, \
                LIM_WRIST_B, LIM_WRIST_F, \
                PORT_BASE, PORT_ELBOW, \
                PORT_HANDH_A, PORT_HANDH_B, \
                PORT_HANDV_A, PORT_HANDV_B, \
                PORT_SHOULD, \
                PORT_TWISTH_A, PORT_TWISTH_B, \
                PORT_TWISTV_A, PORT_TWISTV_B, \
                PORT_WRIST

from brain_consts import PUBLISH_RATE

class ArmClass():
    """
    アームを動かすためのクラス
    """

    def __init__(self):

        # initialize gpio
        self.pic = pigpio.pi()
        self.pic.set_mode(PORT_HANDH_A, pigpio.OUTPUT)
        self.pic.set_mode(PORT_HANDH_B, pigpio.OUTPUT)
        self.pic.set_mode(PORT_HANDV_A, pigpio.OUTPUT)
        self.pic.set_mode(PORT_HANDV_B, pigpio.OUTPUT)
        self.pic.set_mode(PORT_TWISTH_A, pigpio.OUTPUT)
        self.pic.set_mode(PORT_TWISTH_B, pigpio.OUTPUT)
        self.pic.set_mode(PORT_TWISTV_A, pigpio.OUTPUT)
        self.pic.set_mode(PORT_TWISTV_B, pigpio.OUTPUT)

        # パブリッシャーの準備
        self.pub_arm = rospy.Publisher('arm', arm, queue_size=100)
        # messageのインスタンスを作る
        self.msg_arm = arm()
        # index
        self.frame_id = 0
        # モード前回値
        self.mode_old = Mode.UNKNOWN

    def callback(self, armmes):
        """
        メッセージを受信したときに呼び出し
        """

        print('frame_id = %d ' % armmes.frame_id)

        #モード変更確認
        self.grubMotion(armmes.mode)

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


    def clear_Msg(self):
        """
        メッセージ初期化
        """
        self.msg_arm.is_arm_move = False
        self.msg_arm.is_twistv_ulim = False
        self.msg_arm.is_twistv_dlim = False
        self.msg_arm.is_twisth_flim = False
        self.msg_arm.is_twisth_blim = False
        self.msg_arm.is_handv_ulim = False
        self.msg_arm.is_handv_dlim = False
        self.msg_arm.is_handh_flim = False
        self.msg_arm.is_handh_blim = False

    def publishData(self):
        """
        データ送信
        """
        # clear
        self.clear_Msg()
        self.msg_arm.frame_id = self.frame_id
        # publishする関数
        self.pub_arm.publish(self.msg_arm)
        self.frame_id += 1

def printDebug(message):
    """
    デバッグメッセージを表示
    """
    if DEBUG_ARM is True:
        print(message)
    else:
        pass

def arm_py():
    """
    アームのメイン
    """

    armc = ArmClass()
    r = rospy.Rate(PUBLISH_RATE)
    rospy.init_node('arm_py_node', anonymous=True)
    rospy.Subscriber('arm', arm, armc.callback, queue_size=1)
    printDebug("start_arm")
    # ctl +　Cで終了しない限りwhileループでpublishし続ける
    while not rospy.is_shutdown():
        # publishする関数
        armc.publishData()
        #
        r.sleep()

if __name__ == '__main__':
    arm_py()
