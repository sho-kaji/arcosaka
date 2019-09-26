#!/usr/bin/python
# -*- coding: utf-8 -*-
# pylint: disable=E1101,C0325
"""
アーム
"""

import rospy
import mortor


from arc2019.msg import arm

from params import MODE, TARGET

from arm_consts import \
    DEBUG_ARM, \
    LIM_BASE_L, LIM_BASE_R, \
    LIM_ELBOW_B, LIM_ELBOW_F, \
    LIM_SHOULD_B, LIM_SHOULD_F, \
    CHANNEL_ELBOW, \
    CHANNEL_SHOULD, \
    CHANNEL_BASE, \
    PORT_TWISTV_A, PORT_TWISTV_B, \
    PORT_TWISTH_A, PORT_TWISTH_B, \
    PORT_HANDV_A, PORT_HANDV_B, \
    PORT_HANDH_A, PORT_HANDH_B, \
    PORTS_ARM

from brain_consts import CYCLES

class ArmClass(object):
    """
    アームを動かすためのクラス
    """

    def __init__(self):

        # MortorClass
        self.mmc = mortor.MortorClass()

        # initialize port
        for key, val in PORTS_ARM.items():
            self.mmc.pic.set_mode(key, val)

        # パブリッシャーの準備
        self.pub_arm = rospy.Publisher('arm', arm, queue_size=100)

        # モード今回値
        self.mode_now = MODE.UNKNOWN
        self.target_now = TARGET.UNKNOWN

        self.elbow_req_o = 0 # 肘モーター要求値前回値
        self.should_req_o = 0 # 肩モーター要求値前回値
        self.handx_req_o = 0 # ハンド指定位置X前回値
        self.handy_req_o = 0 # ハンド指定位置Y前回値
        self.handz_req_o = 0 # ハンド指定位置Z前回値
        self.twistx_req_o = 0 # ねじ切りハンド指定位置X前回値
        self.twistz_req_o = 0 # ねじ切りハンド指定位置Z前回値

        self.is_arm_move = False
        self.is_arm_call = False
        self.elbow_o = 0 # 肘モーター
        self.should_o = 0 # 肩モーター
        self.base_o = 0 # 土台モーター
        self.twistv_o = 0 # ねじ切り垂直モーター
        self.twisth_o = 0 # ねじ切り水平モーター
        self.handv_o = 0 # ハンド垂直モーター
        self.handh_o = 0 # ハンド水平モーター

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

    def move_elbow(self, elbow):
        """
        肘
        """
        self.is_arm_move = True
        self.mmc.move_servo(CHANNEL_ELBOW, elbow)
        self.is_arm_move = False

    #end move_elbow

    def move_should(self, should):
        """
        肩
        """
        self.is_arm_move = True
        self.mmc.move_servo(CHANNEL_SHOULD, should)
        self.is_arm_move = False

    #end move_should

    def move_base(self, base):
        """
        土台
        """
        self.is_arm_move = True
        self.mmc.move_servo(CHANNEL_BASE, base)
        self.is_arm_move = False

    #end move_base

    def move_twistv(self, twistv):
        """
        ねじ切り垂直
        """
        self.is_arm_move = True
        self.mmc.move_step(PORT_TWISTV_A, PORT_TWISTV_B, twistv)
        self.is_arm_move = False

    #end move_twistv

    def move_twisth(self, twisth):
        """
        ねじ切り水平
        """
        self.is_arm_move = True
        self.mmc.move_step(PORT_TWISTH_A, PORT_TWISTH_B, twisth)
        self.is_arm_move = False

    #end move_twisth

    def move_handv(self, handv):
        """
        ハンド垂直
        """
        self.is_arm_move = True
        self.mmc.move_step(PORT_HANDV_A, PORT_HANDV_B, handv)
        self.is_arm_move = False

    #end move_handv

    def move_handh(self, handh):
        """
        ハンド水平
        """
        self.is_arm_move = True
        self.mmc.move_step(PORT_HANDH_A, PORT_HANDH_B, handh)
        self.is_arm_move = False

    #end move_handh

    def clear_msg(self):
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
    #end clear_msg


    def publish_data(self):
        """
        データ送信
        """
        # clear
        self.clear_msg()
        self.msg_arm.frame_id = self.frame_id
        # 送信データ追加開始

        self.msg_arm.is_arm_move = self.is_arm_move
        self.msg_arm.is_twistv_ulim = False
        self.msg_arm.is_twistv_dlim = False
        self.msg_arm.is_twisth_flim = False
        self.msg_arm.is_twisth_blim = False
        self.msg_arm.is_handv_ulim = False
        self.msg_arm.is_handv_dlim = False
        self.msg_arm.is_handh_flim = False
        self.msg_arm.is_handh_blim = False

        # 送信データ追加終了

        # publishする関数
        self.pub_arm.publish(self.msg_arm)
        self.frame_id += 1
    #end publish_data

def arm_py():
    """
    アームのメイン
    """

    armc = ArmClass()
    rrate = rospy.Rate(CYCLES)
    rospy.init_node('arm_py_node', anonymous=True)
    print("start_arm")
    # ctl +　Cで終了しない限りwhileループでpublishし続ける
    while not rospy.is_shutdown():
        # publishする関数
        armc.publish_data()
        #
        rrate.sleep()
#end arm_py
