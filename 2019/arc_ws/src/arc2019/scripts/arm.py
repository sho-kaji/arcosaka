#!/usr/bin/python
# -*- coding: utf-8 -*-
# pylint: disable=E1101,C0325
"""
アーム
"""

import rospy

import mortor

from arc2019.msg import arm
from arc2019.msg import client
from arc2019.msg import brain

from params import MODE,TARGET

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

from mortor_consts import \
                STEP_1PULSE

from brain_consts import CYCLES

class ArmClass(object):
    """
    アームを動かすためのクラス
    """

    def __init__(self):

        self.mortorc = mortor.MortorClass()

        # initialize port
        for key, val in PORTS_ARM.items():
            self.mortorc.pic.set_mode(key, val)

        # パブリッシャーの準備
        self.pub_arm = rospy.Publisher('arm', arm, queue_size=100)
        # messageのインスタンスを作る
        self.msg_arm = arm()
        # index
        self.frame_id = 0

        # MortorClass
        self.mmc = mortor.MortorClass()

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

    def callback(self, brain_mes):
        """
        メッセージを受信したときに呼び出し
        """

        self.is_arm_move = False
        self.is_arm_call = True

        #モード変更確認
        self.modechange(brain_mes.mode_id, brain_mes.target_id)

        #関数コール
        if self.target_now == TARGET.GRASS: #草刈りモード時
            self.mode_grass()

        elif self.target_now == TARGET.SIDE_SPROUT: #芽かきモード時
            self.mode_sprout()

        elif self.target_now == TARGET.TOMATO: #収穫モード時
            self.mode_tomato()

        else:
            self.mc.endfnc()
        

        #今回値保存ここから
        self.elbow_req_o = brain_mes.elbow_req # 肘モーター要求値
        self.should_req_o = brain_mes.should_req # 肩モーター要求値
        self.handx_req_o = brain_mes.handx_req # ハンド指定位置X
        self.handy_req_o = brain_mes.handy_req # ハンド指定位置Y
        self.handz_req_o = brain_mes.handz_req # ハンド指定位置Z
        self.twistx_req_o = brain_mes.twistx_req # ねじ切りハンド指定位置X
        self.twistz_req_o = brain_mes.twistz_req # ねじ切りハンド指定位置Z
        # 今回値保存ここまで

        #区切り
        self.is_arm_call = False
        print("==============================")
    # end callback

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

    def mode_grass(self):
        """
        草刈り
        """
        if self.mode_now == MODE.AUTO:
            pass
        elif self.mode_now == MODE.MANUAL:
            self.move_base(self.brain_mes.base_req)
            self.move_should(self.brain_mes.should_req)
            self.move_elbow(self.brain_mes.elbow_req)

    #end mode_grass

    def mode_sprout(self):
        """
        芽かき
        """

    #end mode_sprout

    def mode_tomato(self):
        """
        収穫
        """
        if self.mode_now == MODE.AUTO:
            handx = self.brain_mes.handx_req
            handz = self.brain_mes.handz_req

            handv = handz / STEP_1PULSE
            handh = handx / STEP_1PULSE

        elif self.mode_now == MODE.MANUAL:
            handv = 0
            handh = 0

        else:
            pass
        self.move_handv(handv)
        self.move_handh(handh)

    #end mode_tomato

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

    def move_twist(self, twistv, twisth):
        """
        ねじ切り
        """
        self.is_arm_move = True
        self.mmc.move_step(PORT_TWISTV_A, PORT_TWISTV_B, twistv)
        self.mmc.move_step(PORT_TWISTH_A, PORT_TWISTH_B, twisth)
        self.is_arm_move = False

    #end move_twist

    def move_hand(self, handv, handh):
        """
        ハンド
        """
        self.is_arm_move = True
        self.mmc.move_step(PORT_HANDV_A, PORT_HANDV_B, handv)
        self.mmc.move_step(PORT_HANDH_A, PORT_HANDH_B, handh)
        self.is_arm_move = False

    #end move_hand

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
    rospy.Subscriber('brain', brain, armc.callback, queue_size=1)
    print("start_arm")
    # ctl +　Cで終了しない限りwhileループでpublishし続ける
    while not rospy.is_shutdown():
        # publishする関数
        armc.publish_data()
        #
        rrate.sleep()
#end arm_py

if __name__ == '__main__':
    arm_py()
