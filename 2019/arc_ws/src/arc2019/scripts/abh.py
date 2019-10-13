#!/usr/bin/env python
# -*- coding: utf-8 -*-
# pylint: disable=E1101,C0325
"""
アーム・ボディ・ハンド
"""

import math

import rospy
import pigpio
import mortor
import ina226

from arc2019.msg import brain

from arc2019.msg import arm
from arc2019.msg import body
from arc2019.msg import hand

from params import MODE, TARGET

from abh_consts import *

from brain_consts import \
    CYCLES

from mortor_consts import \
    STEP_1PULSE, DC_DUTY

class AbhClass(object):
    """
    アクチュエータを動かすためのクラス
    """

    def __init__(self):

        # index
        self.frame_id = 0

        # パブリッシャーの準備
        self.pub_arm = rospy.Publisher('arm', arm, queue_size=100)
        self.pub_body = rospy.Publisher('body', body, queue_size=100)
        self.pub_hand = rospy.Publisher('hand', hand, queue_size=100)

        self.mes_arm = arm()
        self.mes_body = body()
        self.mes_hand = hand()

        self.is_arm_move = False
        self.is_body_move = False
        self.is_hand_move = False

        self.is_abh_call = False

        #arm用
        self.elbow_req_o = 0 # 肘モーター要求値
        self.should_req_o = 0 # 肩モーター要求値
        self.handx_req_o = 0 # ハンド指定位置X
        self.handy_req_o = 0 # ハンド指定位置Y
        self.handz_req_o = 0 # ハンド指定位置Z
        self.twistx_req_o = 0 # ねじ切りハンド指定位置X
        self.twistz_req_o = 0 # ねじ切りハンド指定位置Z

        #body用
        self.is_pwoffsw = False


        #hand用

        # MortorClass
        self.mmc = mortor.MortorClass()

        #ina226Class
        self.inac = ina226.Ina226Class()

        # initialize port
        self.pic = pigpio.pi()
        for key, val in PORTS_ABH.items():
            self.mmc.pic.set_mode(key, val)

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

        self.elbow_o = 0 # 肘モーター
        self.should_o = 0 # 肩モーター
        self.base_o = 0 # 土台モーター
        self.twistv_o = 0 # ねじ切り垂直モーター
        self.twisth_o = 0 # ねじ切り水平モーター
        self.handv_o = 0 # ハンド垂直モーター
        self.handh_o = 0 # ハンド水平モーター

        self.elbow_req_o = 0 # 肘モーター要求値前回値

    #end __init__

    def posinit(self):
        """
        位置初期化
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

    def clear_msg(self):
        """
        メッセージ初期化
        """

        self.mes_arm.frame_id = -1
        self.mes_arm.is_arm_move = False
        self.mes_arm.is_twistv_ulim = False
        self.mes_arm.is_twistv_dlim = False
        self.mes_arm.is_twisth_flim = False
        self.mes_arm.is_twisth_blim = False
        self.mes_arm.is_handv_ulim = False
        self.mes_arm.is_handv_dlim = False
        self.mes_arm.is_handh_flim = False
        self.mes_arm.is_handh_blim = False

        self.mes_body.frame_id = -1
        self.mes_body.is_body_move = False
        self.mes_body.is_pwoffsw = False
        self.mes_body.batt_v = 0.0
        self.mes_body.batt_i = 0.0

        self.mes_hand.frame_id = -1
        self.mes_hand.is_hand_move = False

    #end clear_msg


    def publish_data(self):
        """
        データ送信
        """
        # clear
        self.clear_msg()

        # 送信データ追加開始

        self.mes_arm.frame_id = self.frame_id
        self.mes_arm.is_arm_move = self.is_arm_move
        self.mes_arm.is_twistv_ulim = False
        self.mes_arm.is_twistv_dlim = False
        self.mes_arm.is_twisth_flim = False
        self.mes_arm.is_twisth_blim = False
        self.mes_arm.is_handv_ulim = False
        self.mes_arm.is_handv_dlim = False
        self.mes_arm.is_handh_flim = False
        self.mes_arm.is_handh_blim = False

        self.mes_body.frame_id = self.frame_id
        self.mes_body.is_body_move = self.is_body_move
        self.is_pwoffsw = self.pic.read(PORT_PWOFFSW) is pigpio.HIGH
        self.mes_body.is_pwoffsw = self.is_pwoffsw
        self.mes_body.batt_v = self.inac.read_v()
        self.mes_body.batt_i = self.inac.read_i()

        self.mes_hand.frame_id = self.frame_id
        self.mes_hand.is_hand_move = self.is_hand_move

        # 送信データ追加終了

        # publishする関数
        self.pub_arm.publish(self.mes_arm)
        self.pub_body.publish(self.mes_body)
        self.pub_hand.publish(self.mes_hand)
        self.frame_id += 1
    #end publish_data

    def calc_saturation(self, num_val, num_min, num_max):
        """
        上下限ガード
        """
        if num_min > num_max:
            num_tmp = num_max
            num_max = num_min
            num_min = num_tmp
        if num_val < num_min:
            num_val = num_min
        elif num_max < num_val:
            num_val = num_max

        return num_val
    #end calc_saturation

    def move_elbow(self, elbow):
        """
        肘
        """
        self.is_arm_move = True
        self.mmc.move_servo(CHANNEL_ELBOW, elbow, LIM_ELBOW_B, LIM_ELBOW_F)
        self.is_arm_move = False

    #end move_elbow

    def move_should(self, should):
        """
        肩
        """
        self.is_arm_move = True
        self.mmc.move_servo(CHANNEL_SHOULD, should, LIM_SHOULD_B, LIM_SHOULD_F)
        self.is_arm_move = False

    #end move_should

    def move_base(self, base):
        """
        土台
        """
        self.is_arm_move = True
        self.mmc.move_servo(CHANNEL_BASE, base, LIM_BASE_L, LIM_BASE_R)
        self.is_arm_move = False

    #end move_base

    def move_twistv(self, twistv):
        """
        ねじ切り垂直
        """
        self.is_arm_move = True
        twistv = self.calc_saturation(twistv, LIM_TWISTV_MIN, LIM_TWISTV_MAX)
        self.mmc.move_step(PORT_TWISTV_A, PORT_TWISTV_B, twistv)
        self.is_arm_move = False

    #end move_twistv

    def move_twisth(self, twisth):
        """
        ねじ切り水平
        """
        self.is_arm_move = True
        twisth = self.calc_saturation(twisth, LIM_TWISTH_MIN, LIM_TWISTH_MAX)
        self.mmc.move_step(PORT_TWISTH_A, PORT_TWISTH_B, twisth)
        self.is_arm_move = False

    #end move_twisth

    def move_handv(self, handv):
        """
        ハンド垂直
        """
        self.is_arm_move = True
        handv = self.calc_saturation(handv, LIM_HANDV_MIN, LIM_HANDV_MAX)
        self.mmc.move_step(PORT_HANDV_A, PORT_HANDV_B, handv)
        self.is_arm_move = False

    #end move_handv

    def move_handh(self, handh):
        """
        ハンド水平
        """
        self.is_arm_move = True
        handh = self.calc_saturation(handh, LIM_HANDH_MIN, LIM_HANDH_MAX)
        self.mmc.move_step(PORT_HANDH_A, PORT_HANDH_B, handh)
        self.is_arm_move = False

    #end move_handh

    def move_lid(self, lid):
        """
        蓋
        """
        self.is_body_move = True
        self.mmc.move_servo(CHANNEL_LID, lid, LIM_LID_MIN, LIM_LID_MAX)
        self.is_body_move = False

    #end move_lid

    def move_spray(self, spray):
        """
        散布ファン
        """
        self.is_body_move = True
        spray = self.calc_saturation(spray, OFF_SPRAY, ON_SPRAY)
        self.mmc.move_dc_duty(PORT_SPRAY, -1, spray, 0)
        self.is_body_move = False

    #end move_spray

    def move_blade(self, blade):
        """
        刃
        """
        blade = self.calc_saturation(blade, BLADE_MIMUS, BLADE_PLUS)
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

    def move_hand(self, handm):
        """
        ハンド
        """
        self.is_hand_move = True
        self.mmc.move_servo(CHANNEL_HAND, handm, RELEASE_HAND, CATCH_HAND)
        self.is_hand_move = False

    #end move_hand

    def move_wrist(self, wrist):
        """
        手首
        """
        self.is_hand_move = True
        self.mmc.move_servo(CHANNEL_WRIST, wrist, LIM_WRIST_F, LIM_WRIST_B)
        self.is_hand_move = False

    #end move_wrist

    def move_pluck(self, pluck):
        """
        引抜
        """
        self.is_hand_move = True
        self.mmc.move_servo(CHANNEL_PLUCK, pluck, OFF_PLUCK, ON_PLUCK)
        self.is_hand_move = False

    #end move_pluck

    def move_grab(self, grab):
        """
        枝掴み
        """
        self.is_hand_move = True
        self.mmc.move_servo(CHANNEL_GRAB, grab, RELEASE_GRAB, CATCH_GRAB)
        self.is_hand_move = False

    #end move_grab

    def move_twist(self, twist):
        """
        枝ねじり
        """
        self.is_hand_move = True
        self.mmc.move_servo(CHANNEL_TWIST, twist, OFF_TWIST, ON_TWIST)
        self.is_hand_move = False

    #end move_twist

    def move_attach_r(self, attach_r):
        """
        添え手右
        """
        self.is_hand_move = True
        self.mmc.move_servo(CHANNEL_ATTACH_RR, attach_r, LIM_ATTACH_RL, LIM_ATTACH_RR)
        self.is_hand_move = False

    #end move_attach_r

    def move_attach_l(self, attach_l):
        """
        添え手左
        """
        self.is_hand_move = True
        self.mmc.move_servo(CHANNEL_ATTACH_LR, attach_l, LIM_ATTACH_LL, LIM_ATTACH_LR)
        self.is_hand_move = False

    #end move_attach_l

    def callback(self, brain_mes):
        """
        メッセージを受信したときに呼び出し
        """

        self.is_abh_call = True

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
        self.is_abh_call = False
        print("==============================")
    # end callback

    def mode_grass(self):
        """
        草刈り
        土台 => 肩 => ハンド => 引抜 => (肘) => 手首 => 散布ファン => ハンド => 引抜 =>
        => 肩 => 土台(66.7%) => 蓋 => 手首 => 肘 => ハンド => 蓋 => 刃
        """
        if self.mode_now == MODE.AUTO:
            #土台モーターの角度計算
            tmp_base = 0
            #肩モーターの角度計算
            tmp_should = 0
            #手首モーターの角度計算
            tmp_wrist = 0
            #モーター動作
            self.move_base(tmp_base)
            self.move_should(tmp_should)
            self.move_hand(RELEASE_HAND)
            self.move_pluck(ON_PLUCK)
            #self.move_elbow()
            self.move_wrist(tmp_wrist)
            self.move_spray(ON_SPRAY)
            self.move_spray(OFF_SPRAY)
            self.move_hand(CATCH_HAND)
            self.move_pluck(ON_PLUCK)
            self.move_should(0)
            self.move_base((2.0/3.0)*100)
            self.move_wrist(100)
            self.move_elbow(100)
            self.move_lid(100)
            self.move_hand(RELEASE_HAND)
            self.move_lid(0)
            self.move_blade(BLADE_PLUS)
            self.move_blade(BLADE_NONE)


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
        if self.mode_now == MODE.AUTO: # 自動モード
            tmp_twistx = self.brain_mes.twistx_req
            tmp_twistz = self.brain_mes.twistz_req

            tmp_twistv = tmp_twistz / STEP_1PULSE
            tmp_twisth = tmp_twistx / STEP_1PULSE

            self.armc.move_twistv(tmp_twistv)
            self.armc.move_twisth(tmp_twisth)
            self.handc.move_attach_l(LIM_ATTACH_LR)
            self.handc.move_attach_r(LIM_ATTACH_RL)
            self.handc.move_grab(CATCH_GRAB)
            self.handc.move_twist(ON_TWIST)
            self.handc.move_grab(RELEASE_GRAB)
            self.handc.move_attach_l(LIM_ATTACH_LL)
            self.handc.move_attach_r(LIM_ATTACH_RR)
            self.armc.move_twisth(LIM_TWISTH_MIN)

        elif self.mode_now == MODE.MANUAL: # 手動モード
            twistv = 0 #brainから値が来るはず
            twisth = 0

        else:
            pass

    #end mode_sprout

    def mode_tomato(self):
        """
        収穫
        ハンド垂直 => ハンド水平 => ハンド => 手首
         =>ハンド => ハンド水平 => 手首
        """
        if self.mode_now == MODE.AUTO: # 自動モード
            handx = self.brain_mes.handx_req
            handz = self.brain_mes.handz_req

            handv = handz / STEP_1PULSE
            handh = handx / STEP_1PULSE

            self.armc.move_handv(handv) # ハンド垂直
            self.armc.move_handh(handh) # ハンド水平
            self.handc.move_hand(CATCH_HAND) # ハンド
            self.handc.move_wrist(LIM_WRIST_F) #手首
            self.handc.move_hand(RELEASE_HAND) # ハンド
            self.armc.move_handh(handh) # ハンド水平
            self.handc.move_wrist(LIM_WRIST_B) #手首
            self.armc.move_handh(LIM_HANDH_MIN) # ハンド水平

        elif self.mode_now == MODE.MANUAL: # 手動モード
            handv = LIM_HANDV_MIN #brainから値が来るはず
            handh = LIM_HANDH_MIN

        else:
            pass

    #end mode_tomato
def abh_py():
    """
    アーム・ボディ・ハンドのメイン
    """
    abhc = AbhClass()

    rospy.init_node('abh_py_node', anonymous=True)
    rrate = rospy.Rate(CYCLES)
    rospy.Subscriber('brain', brain, abhc.callback, queue_size=1)
    print("start_abh")
    # ctl + Cで終了しない限りwhileループでpublishし続ける
    while not rospy.is_shutdown():
        abhc.publish_data()
        rrate.sleep()
#end abh_py
if __name__ == '__main__':
    abh_py()
