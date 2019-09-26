#!/usr/bin/python
# -*- coding: utf-8 -*-
# pylint: disable=E1101,C0325
"""
アーム・ボディ・ハンド
"""

import rospy

import arm
import body
import hand

from arc2019.msg import brain

from params import MODE, TARGET
from brain_consts import \
    CYCLES

from mortor_consts import \
    STEP_1PULSE

class AbhClass(object):
    """
    アームを動かすためのクラス
    """

    def __init__(self):

        self.armc = arm.ArmClass()
        self.bodyc = body.BodyClass()
        self.handc = hand.HandClass()

        # index
        self.frame_id = 0

        self.is_arm_move = False
        self.is_arm_call = False
        self.is_body_move = False
        self.is_body_call = False
        self.is_hand_move = False
        self.is_hand_call = False

        #arm用
        self.elbow_req_o = 0 # 肘モーター要求値
        self.should_req_o = 0 # 肩モーター要求値
        self.handx_req_o = 0 # ハンド指定位置X
        self.handy_req_o = 0 # ハンド指定位置Y
        self.handz_req_o = 0 # ハンド指定位置Z
        self.twistx_req_o = 0 # ねじ切りハンド指定位置X
        self.twistz_req_o = 0 # ねじ切りハンド指定位置Z

        self.mode_now = MODE.UNKNOWN
        self.target_now = TARGET.UNKNOWN

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
            pass


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
            #何か処理

        if self.target_now != target:
            self.target_now = target
            #何か処理

        self.armc.mode_now(mode, target)
        self.bodyc.mode_now(mode, target)
        self.handc.mode_now(mode, target)

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
        ねじ切り垂直 => ねじ切り水平 => 添え手右・添え手左 => 枝掴み =>
        => 枝ねじり => 枝掴み => 添え手右・添え手左 => ねじ切り水平
        """


    #end mode_sprout

    def mode_tomato(self):
        """
        収穫
        ハンド垂直 => ハンド水平 => ハンド => 手首 => ハンド => ハンド水平 => ハンド垂直
        """
        if self.mode_now == MODE.AUTO: # 自動モード
            handx = self.brain_mes.handx_req
            handz = self.brain_mes.handz_req

            handv = handz / STEP_1PULSE
            handh = handx / STEP_1PULSE

        elif self.mode_now == MODE.MANUAL: # 手動モード
            handv = 0
            handh = 0

        else:
            pass
        self.armc.move_handv(handv) # ハンド垂直
        self.armc.move_handh(handh) # ハンド水平
        self.handc.move_hand(CATCH_HAND) # ハンド




    #end mode_tomato

def abh_py():
    """
    アーム・ボディ・ハンドのメイン
    """
    abhc = AbhClass()

    rrate = rospy.Rate(CYCLES)
    rospy.init_node('abh_py_node', anonymous=True)
    rospy.Subscriber('brain', brain, abhc.callback, queue_size=1)
    print("start_abh")
    # ctl +　Cで終了しない限りwhileループでpublishし続ける
    while not rospy.is_shutdown():
        rrate.sleep()
#end arm_py

if __name__ == '__main__':
    abh_py()
