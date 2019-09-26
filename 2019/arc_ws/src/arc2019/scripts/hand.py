#!/usr/bin/env python
# -*- coding: utf-8 -*-
# pylint: disable=E1101,C0325
"""
ハンド
"""

import rospy
import pigpio
import mortor

from arc2019.msg import hand

from params import MODE, TARGET

from hand_consts import \
    DEBUG_HAND, \
    CATCH_HAND, RELEASE_HAND, \
    CHANNEL_HAND, \
    LIM_WRIST_F, LIM_WRIST_B, \
    CHANNEL_WRIST, \
    ON_PLUCK, OFF_PLUCK, \
    CHANNEL_PLUCK, \
    CATCH_GRAB, RELEASE_GRAB, \
    CHANNEL_GRAB, \
    ON_TWIST, OFF_TWIST, \
    CHANNEL_TWIST, \
    LIM_ATTACH_RL, LIM_ATTACH_RR, \
    CHANNEL_ATTACH_RR, \
    LIM_ATTACH_LL, LIM_ATTACH_LR, \
    CHANNEL_ATTACH_LR, \
    PORTS_HAND

from mortor_consts import \
    DC_DUTY

from brain_consts import CYCLES

class HandClass(object):
    """
    ボディを動かすためのクラス
    """

    def __init__(self):

        # MortorClass
        self.mmc = mortor.MortorClass()

        # initialize port
        self.pic = pigpio.pi()
        for key, val in PORTS_HAND.items():
            self.mmc.pic.set_mode(key, val)

        # パブリッシャーの準備
        self.pub_arm = rospy.Publisher('hand', hand, queue_size=100)

        # モード今回値
        self.mode_now = MODE.UNKNOWN
        self.target_now = TARGET.UNKNOWN

        self.elbow_req_o = 0 # 肘モーター要求値前回値

        self.is_hand_move = False
        self.is_hand_call = False
        self.is_pwoffsw = False

    #end __init__

    def posinit(self):
        """
        アーム位置初期化
        """
        pass

    def modechange(self, mode, target):
        """
        モード変更確認
        """
        if self.mode_now != mode:
            self.mode_now = mode
            self.mmc.endfnc()
            #何か処理

        if self.target_now != target:
            self.target_now = target
            self.mmc.endfnc()
            #何か処理

    #end modechange


    def move_hand(self, handm):
        """
        ハンド
        """
        self.is_hand_move = True
        self.mmc.move_servo(CHANNEL_HAND, handm)
        self.is_hand_move = False

    #end move_hand

    def move_wrist(self, wrist):
        """
        手首
        """
        self.is_hand_move = True
        self.mmc.move_servo(CHANNEL_WRIST, wrist)
        self.is_hand_move = False

    #end move_wrist

    def move_pluck(self, pluck):
        """
        引抜
        """
        self.is_hand_move = True
        self.mmc.move_servo(CHANNEL_PLUCK, pluck)
        self.is_hand_move = False

    #end move_pluck

    def move_grab(self, grab):
        """
        枝掴み
        """
        self.is_hand_move = True
        self.mmc.move_servo(CHANNEL_GRAB, grab)
        self.is_hand_move = False

    #end move_grab

    def move_twist(self, twist):
        """
        枝ねじり
        """
        self.is_hand_move = True
        self.mmc.move_servo(CHANNEL_TWIST, twist)
        self.is_hand_move = False

    #end move_twist

    def move_attach_r(self, attach_r):
        """
        添え手右
        """
        self.is_hand_move = True
        self.mmc.move_servo(CHANNEL_ATTACH_RR, attach_r)
        self.is_hand_move = False

    #end move_attach_r

    def move_attach_l(self, attach_l):
        """
        添え手左
        """
        self.is_hand_move = True
        self.mmc.move_servo(CHANNEL_ATTACH_LR, attach_l)
        self.is_hand_move = False

    #end move_attach_l

    def clear_msg(self):
        """
        メッセージ初期化
        """
        self.msg_hand.is_hand_move = False

    #end clear_msg


    def publish_data(self):
        """
        データ送信
        """
        # clear
        self.clear_msg()
        self.msg_hand.frame_id = self.frame_id
        # 送信データ追加開始

        self.msg_hand.is_hand_move = self.is_hand_move

        # 送信データ追加終了

        # publishする関数
        self.pub_hand.publish(self.msg_hand)
        self.frame_id += 1
    #end publish_data

def hand_py():
    """
    ボディのメイン
    """

    handc = HandClass()
    rrate = rospy.Rate(CYCLES)
    rospy.init_node('hand_py_node', anonymous=True)
    print("start_hand")
    # ctl +　Cで終了しない限りwhileループでpublishし続ける
    while not rospy.is_shutdown():
        # publishする関数
        handc.publish_data()
        #
        rrate.sleep()
#end hand_py
