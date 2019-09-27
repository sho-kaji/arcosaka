#!/usr/bin/env python
# -*- coding: utf-8 -*-
# pylint: disable=E1101,C0325
"""
ボディ
"""

import rospy
import pigpio
import mortor

from arc2019.msg import body

from params import MODE, TARGET

from body_consts import \
    DEBUG_BODY, \
    CHANNEL_LID, \
    PORT_SPRAY, \
    PORT_BLADE_A, PORT_BLADE_B, \
    BLADE_MIMUS, BLADE_NONE, BLADE_PLUS, \
    PORT_PWOFFSW, \
    PORTS_BODY

from mortor_consts import \
    DC_DUTY

from brain_consts import CYCLES

class BodyClass(object):
    """
    ボディを動かすためのクラス
    """

    def __init__(self):

        # MortorClass
        self.mmc = mortor.MortorClass()

        # initialize port
        self.pic = pigpio.pi()
        for key, val in PORTS_BODY.items():
            self.mmc.pic.set_mode(key, val)

        # パブリッシャーの準備
        self.pub_arm = rospy.Publisher('body', body, queue_size=100)

        # モード今回値
        self.mode_now = MODE.UNKNOWN
        self.target_now = TARGET.UNKNOWN

        self.elbow_req_o = 0 # 肘モーター要求値前回値

        self.is_body_move = False
        self.is_body_call = False
        self.is_pwoffsw = False

    #end __init__

    def posinit(self):
        """
        アーム位置初期化
        蓋を閉める => 
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


    def move_lid(self, lid):
        """
        蓋
        """
        self.is_body_move = True
        self.mmc.move_servo(CHANNEL_LID, lid)
        self.is_body_move = False

    #end move_lid

    def move_spray(self, spray):
        """
        散布
        """
        self.is_body_move = True
        self.mmc.move_dc_duty(PORT_SPRAY, -1, spray, 0)
        self.is_body_move = False

    #end move_spray

    def move_blade(self, blade):
        """
        刃
        """

        blade_p = blade * DC_DUTY
        if blade_p < 0:
            blade_p = 0

        blade_m = -(blade * DC_DUTY)
        if blade_m < 0:
            blade_m = 0

        self.is_body_move = True
        self.mmc.move_dc_duty(PORT_BLADE_A, PORT_BLADE_B, blade_p, blade_m)
        self.is_body_move = False

    #end move_blade


    def clear_msg(self):
        """
        メッセージ初期化
        """
        self.msg_body.is_body_move = False

    #end clear_msg


    def publish_data(self):
        """
        データ送信
        """
        # clear
        self.clear_msg()
        self.msg_body.frame_id = self.frame_id
        # 送信データ追加開始

        self.msg_body.is_body_move = self.is_body_move
        self.is_pwoffsw = self.pic.read(PORT_PWOFFSW) is pigpio.HIGH
        self.msg_body.is_pwoffsw = self.is_pwoffsw

        # 送信データ追加終了

        # publishする関数
        self.pub_body.publish(self.msg_body)
        self.frame_id += 1
    #end publish_data

    def body_py(self):
        """
        ボディのメイン
        """

        rrate = rospy.Rate(CYCLES)
        print("start_body")
        # ctl +　Cで終了しない限りwhileループでpublishし続ける
        while not rospy.is_shutdown():
            # publishする関数
            self.publish_data()
            #
            rrate.sleep()
    #end body_py

rospy.init_node('body_py_node', anonymous=True)
