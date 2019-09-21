#!/usr/bin/python
# -*- coding: utf-8 -*-
# pylint: disable=E1101
"""
アーム
"""

import rospy

import mortor

from arc2019.msg import arm
from arc2019.msg import client
from arc2019.msg import brain

from params import MODE,TARGET

# from params import TARGET

from arm_consts import \
                DEBUG_ARM, \
                LIM_BASE_L, LIM_BASE_R, \
                LIM_ELBOW_B, LIM_ELBOW_F, \
                LIM_SHOULD_B, LIM_SHOULD_F, \
                PORTS_ARM

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

        self.elbow = 0 # 肘モーター
        self.should = 0 # 肩モーター
        self.base = 0 # 土台モーター
        self.twistv = 0 # ねじ切り垂直モーター
        self.twisth = 0 # ねじ切り水平モーター
        self.handv = 0 # ハンド垂直モーター
        self.handh = 0 # ハンド水平モーター



    def callback(self, brainmes):
        """
        メッセージを受信したときに呼び出し
        """

        #モード変更確認
        self.modechange(brainmes.mode, brainmes.target)

        #関数コール
        if self.target_now == TARGET.GRASS: #草刈りモード時
            if self.mode_now == MODE.AUTO:
                if self.elbow_req_o != brainmes.elbow_req:
                    pass
            elif self.mode_now == MODE.MANUAL:
                self.elbow_req_o = brainmes.elbow_req




        #今回値保存
        self.elbow_req_o = brainmes.elbow_req # 肘モーター要求値
        self.should_req_o = brainmes.should_req # 肩モーター要求値
        self.handx_req_o = brainmes.handx_req # ハンド指定位置X
        self.handy_req_o = brainmes.handy_req # ハンド指定位置Y
        self.handz_req_o = brainmes.handz_req # ハンド指定位置Z
        self.twistx_req_o = brainmes.twistx_req # ねじ切りハンド指定位置X
        self.twistz_req_o = brainmes.twistz_req # ねじ切りハンド指定位置Z

        #区切り
        print("==============================")

    def modechange(self, mode, target):
        """
        モード変更確認
        """
        if self.mode_now != mode:
            self.mode_now = mode
            #何か処理

        if self.target_now != target:
            self.target_now = target
            #何か処理



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

    def publish_data(self):
        """
        データ送信
        """
        # clear
        self.clear_msg()
        self.msg_arm.frame_id = self.frame_id
        # 送信データ追加開始

        # 送信データ追加終了

        # publishする関数
        self.pub_arm.publish(self.msg_arm)
        self.frame_id += 1

def print_debug(message):
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
    rrate = rospy.Rate(CYCLES)
    rospy.init_node('arm_py_node', anonymous=True)
    rospy.Subscriber('brain', brain, armc.callback, queue_size=1)
    print_debug("start_arm")
    # ctl +　Cで終了しない限りwhileループでpublishし続ける
    while not rospy.is_shutdown():
        # publishする関数
        armc.publish_data()
        #
        rrate.sleep()

if __name__ == '__main__':
    arm_py()
