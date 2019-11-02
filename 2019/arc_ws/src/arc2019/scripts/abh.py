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
from arc2019.msg import client_abh_debug

from arc2019.msg import arm
from arc2019.msg import body
from arc2019.msg import hand

from params import MODE, TARGET

from abh_consts import *

from brain_consts import \
    CYCLES

from mortor_consts import \
    STEPROTATE, \
    STEP_1PULSE, DC_DUTY

from ina226_consts import \
    BATT_VMAX, BATT_VLOW


class AbhClass(object):
    """
    アクチュエータを動かすためのクラス
    """

    def __init__(self, noros = False):

        # index
        self.frame_id = 0

        self.noros = noros

        # パブリッシャーの準備
        if not(self.noros):
            self.pub_arm = rospy.Publisher('arm', arm, queue_size=100)
            self.pub_body = rospy.Publisher('body', body, queue_size=100)
            self.pub_hand = rospy.Publisher('hand', hand, queue_size=100)
        #end if not(noros)

        self.mes_arm = arm()
        self.mes_body = body()
        self.mes_hand = hand()
        self.mes_brain = brain()

        self.is_arm_move = False
        self.is_body_move = False
        self.is_hand_move = False

        self.is_abh_call = False

        # arm用
        self.elbow_req_o = 0  # 肘モーター要求値
        self.should_req_o = 0  # 肩モーター要求値
        self.handx_req_o = 0  # ハンド指定位置X
        self.handy_req_o = 0  # ハンド指定位置Y
        self.handz_req_o = 0  # ハンド指定位置Z
        self.twistx_req_o = 0  # ねじ切りハンド指定位置X
        self.twistz_req_o = 0  # ねじ切りハンド指定位置Z

        # body用
        self.is_pwoffsw = False

        # hand用

        # MortorClass
        self.dmc = mortor.DcMortorClass(
            DEBUG_BODY, (PORT_BLADE_A, PORT_BLADE_B))

        self.stmc_handh = mortor.StepMortorClass(
            DEBUG_ARM, (PORT_HANDH_A, PORT_HANDH_B), (LIM_HANDH_MIN, LIM_HANDH_MAX))
        self.stmc_handv = mortor.StepMortorClass(
            DEBUG_ARM, (PORT_HANDV_A, PORT_HANDV_B), (LIM_HANDV_MIN, LIM_HANDV_MAX))
        self.stmc_twisth = mortor.StepMortorClass(
            DEBUG_ARM, (PORT_TWISTH_A, PORT_TWISTH_B), (LIM_TWISTH_MIN, LIM_TWISTH_MAX))
        self.stmc_twistv = mortor.StepMortorClass(
            DEBUG_ARM, (PORT_TWISTV_A, PORT_TWISTV_B), (LIM_TWISTV_MIN, LIM_TWISTV_MAX))

        self.svmc = mortor.ServoMortorClass(
            (DEBUG_ARM or DEBUG_HAND), TARGET.UNKNOWN)

        # ina226Class
        self.inac = ina226.Ina226Class()

        # initialize port
        self.pic = pigpio.pi()
        self.pic.set_mode(PORT_PWOFFSW, pigpio.INPUT)

        # モード今回値
        self.mode_now = MODE.UNKNOWN
        self.target_now = TARGET.UNKNOWN

        self.elbow_req_o = 0  # 肘モーター要求値前回値
        self.should_req_o = 0  # 肩モーター要求値前回値
        self.handx_req_o = 0  # ハンド指定位置X前回値
        self.handy_req_o = 0  # ハンド指定位置Y前回値
        self.handz_req_o = 0  # ハンド指定位置Z前回値
        self.twistx_req_o = 0  # ねじ切りハンド指定位置X前回値
        self.twistz_req_o = 0  # ねじ切りハンド指定位置Z前回値

        self.elbow_o = 0  # 肘モーター
        self.should_o = 0  # 肩モーター
        self.base_o = 0  # 土台モーター
        self.twistv_o = 0  # ねじ切り垂直モーター
        self.twisth_o = 0  # ねじ切り水平モーター
        self.handv_o = 0  # ハンド垂直モーター
        self.handh_o = 0  # ハンド水平モーター

        self.elbow_req_o = 0  # 肘モーター要求値前回値

        # debug用
        self.arm_grass_hand = 0
        self.arm_grass_pluck = 0
        self.arm_grass_wrist = 0
        self.arm_grass_elbow = 0
        self.arm_grass_sholder = 0
        self.arm_grass_base = 0
        self.arm_grass_lid = 0
        self.arm_grass_splay = 0
        self.arm_grass_blade = 0
        self.arm_crop_hand = 0
        self.arm_crop_wrist = 0
        self.arm_crop_vertical = 0
        self.arm_crop_horizontal = 0
        self.arm_sprout_grub = 0
        self.arm_sprout_twist = 0
        self.arm_sprout_attach_r = 0
        self.arm_sprout_attatch_l = 0
        self.arm_sprout_vertical = 0
        self.arm_sprout_horizontal = 0

    # end __init__

    def posinit(self):
        """
        位置初期化
        """
        if self.target_now == TARGET.GRASS:
            # DCモーター
            self.dmc.posinit()
            # サーボモーター
            for channel in enumerate(CHANNEL_K):
                self.svmc.posinit(channel)

        elif (self.target_now == TARGET.SIDE_SPROUT) or (self.target_now == TARGET.SIDE_SPROUT):
            # ステッピングモーター
            self.stmc_twistv.posinit(STEPROTATE.MINUS)
            self.stmc_twisth.posinit(STEPROTATE.PLUS)
            self.stmc_handv.posinit(STEPROTATE.PLUS)
            self.stmc_handh.posinit(STEPROTATE.PLUS)
            # サーボモーター
            for channel in enumerate(CHANNEL_M):
                self.svmc.posinit(channel)
    # end posinit

    def modechange(self, mode, target):
        """
        モード変更確認
        """
        is_change = False
        if self.mode_now != mode:
            self.mode_now = mode
            is_change = True
            # 何か処理

        if self.target_now != target:
            self.target_now = target
            self.svmc.target = target
            is_change = True
            # 何か処理

        if is_change:
            self.posinit()

    # end modechange

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

    # end clear_msg

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
        batt_i = self.inac.read_i()
        batt_v = self.inac.read_v()
        batt_b = self.inac.read_b()
        self.mes_body.batt_i = batt_i
        self.mes_body.batt_v = batt_v
        self.mes_body.batt = batt_b

        self.mes_hand.frame_id = self.frame_id
        self.mes_hand.is_hand_move = self.is_hand_move

        # 送信データ追加終了

        # publishする関数
        if not(self.noros):
            self.pub_arm.publish(self.mes_arm)
            self.pub_body.publish(self.mes_body)
            self.pub_hand.publish(self.mes_hand)
        #end if not(self.noros)
        self.frame_id += 1

    # end publish_data

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
    # end calc_saturation

    def move_elbow(self, elbow):
        """
        肘
        """
        self.is_arm_move = True
        print("elbow    = %d" % elbow)
        self.svmc.move_servo(CHANNEL_ELBOW, elbow)
        self.is_arm_move = False

    # end move_elbow

    def move_should(self, should):
        """
        肩
        """
        self.is_arm_move = True
        print("should   = %d" % should)
        self.svmc.move_servo(CHANNEL_SHOULD, should)
        self.is_arm_move = False

    # end move_should

    def move_base(self, base):
        """
        土台
        """
        self.is_arm_move = True
        print("base     = %d" % base)
        self.svmc.move_servo(CHANNEL_BASE, base)
        self.is_arm_move = False

    # end move_base

    def move_twistv(self, twistv):
        """
        ねじ切り垂直
        """
        self.is_arm_move = True
        print("twistv   = %d" % twistv)
        self.stmc_twistv.move_step(twistv)
        self.is_arm_move = False

    # end move_twistv

    def move_twisth(self, twisth):
        """
        ねじ切り水平
        """
        self.is_arm_move = True
        print("twisth   = %d" % twisth)
        self.stmc_twisth.move_step(twisth)
        self.is_arm_move = False

    # end move_twisth

    def move_handv(self, handv):
        """
        ハンド垂直
        """
        self.is_arm_move = True
        print("handv    = %d" % handv)
        self.stmc_handv.move_step(handv)
        self.is_arm_move = False

    # end move_handv

    def move_handh(self, handh):
        """
        ハンド水平
        """
        self.is_arm_move = True
        print("handh    = %d" % handh)
        self.stmc_handh.move_step(handh)
        self.is_arm_move = False

    # end move_handh

    def move_lid(self, lid):
        """
        蓋
        """
        self.is_body_move = True
        print("lid      = %d" % lid)
        self.svmc.move_servo(CHANNEL_LID, lid)
        self.is_body_move = False

    # end move_lid

    def move_spray(self, spray):
        """
        散布ファン
        """
        self.is_body_move = True
        spray = self.calc_saturation(spray, OFF_SPRAY, ON_SPRAY)
        print("spray    = %d" % spray)
        self.dmc.move_dc_duty(PORT_SPRAY, -1, spray, 0)
        self.is_body_move = False

    # end move_spray

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
        print("blade    = %d,%d" % (blade_p, blade_m))
        self.dmc.move_dc_duty(PORT_BLADE_A, PORT_BLADE_B, blade_p, blade_m)
        self.is_body_move = False

    # end move_blade

    def move_hand(self, handm):
        """
        ハンド
        """
        self.is_hand_move = True
        print("handm    = %d" % handm)
        self.svmc.move_servo(CHANNEL_HAND, handm)
        self.is_hand_move = False

    # end move_hand

    def move_wrist(self, wrist):
        """
        手首
        """
        self.is_hand_move = True
        print("wrist    = %d" % wrist)
        self.svmc.move_servo(CHANNEL_WRIST, wrist)
        self.is_hand_move = False

    # end move_wrist

    def move_pluck(self, pluck):
        """
        引抜
        """
        self.is_hand_move = True
        print("pluck    = %d" % pluck)
        self.svmc.move_servo(CHANNEL_PLUCK, pluck)
        self.is_hand_move = False

    # end move_pluck

    def move_grab(self, grab):
        """
        枝掴み
        """
        self.is_hand_move = True
        print("grab     = %d" % grab)
        self.svmc.move_servo(CHANNEL_GRAB, grab)
        self.is_hand_move = False

    # end move_grab

    def move_twist(self, twist):
        """
        枝ねじり
        """
        self.is_hand_move = True
        print("twist    = %d" % twist)
        self.svmc.move_servo(CHANNEL_TWIST, twist)
        self.is_hand_move = False

    # end move_twist

    def move_attach_r(self, attach_r):
        """
        添え手右
        """
        self.is_hand_move = True
        print("attach_r = %d" % attach_r)
        self.svmc.move_servo(CHANNEL_ATTACH_RR, attach_r)
        self.is_hand_move = False

    # end move_attach_r

    def move_attach_l(self, attach_l):
        """
        添え手左
        """
        self.is_hand_move = True
        print("attach_l = %d" % attach_l)
        self.svmc.move_servo(CHANNEL_ATTACH_LR, attach_l)
        self.is_hand_move = False

    # end move_attach_l

    def callback(self, mes_brain):
        """
        メッセージを受信したときに呼び出し
        """

        self.is_abh_call = True
        self.mes_brain = mes_brain

        # モード変更確認
        self.modechange(mes_brain.mode_id, mes_brain.target_id)
        if self.mode_now == MODE.AUTO:
            # 関数コール
            if self.target_now == TARGET.GRASS:  # 草刈りモード時
                self.mode_grass()

            elif self.target_now == TARGET.SIDE_SPROUT:  # 芽かきモード時
                self.mode_sprout()

            elif self.target_now == TARGET.TOMATO:  # 収穫モード時
                self.mode_tomato()

            else:
                pass
        elif self.mode_now == MODE.MANUAL:
            pass
        else:
            pass

        # 今回値保存ここから

        self.elbow_req_o = mes_brain.elbow_req  # 肘モーター要求値
        self.should_req_o = mes_brain.should_req  # 肩モーター要求値
        self.handx_req_o = mes_brain.handx_req  # ハンド指定位置X
        self.handy_req_o = mes_brain.handy_req  # ハンド指定位置Y
        self.handz_req_o = mes_brain.handz_req  # ハンド指定位置Z
        self.twistx_req_o = mes_brain.twistx_req  # ねじ切りハンド指定位置X
        self.twistz_req_o = mes_brain.twistz_req  # ねじ切りハンド指定位置Z
        # 今回値保存ここまで

        # 区切り
        self.is_abh_call = False
        print("brain2abh==============================")
    # end callback

    def callback_debug(self, mes_cad):
        """
        メッセージを受信したときに呼び出し
        """
        print("brain2abh_debug=============================")
        self.is_abh_call = True
        #self.mes_cad = mes_cad

        # モード変更確認
        self.modechange(mes_cad.mode, mes_cad.target)
        print("  MODE=%s" % MODE(self.mode_now).name)
        print("TARGET=%s" % TARGET(self.target_now).name)
        if self.mode_now == MODE.DEBUG:
            # 関数コール
            if self.target_now == TARGET.GRASS:  # 草刈りモード時
                if self.arm_grass_hand != mes_cad.arm_grass_hand:
                    self.arm_grass_hand = mes_cad.arm_grass_hand
                    self.move_hand(self.arm_grass_hand)

                if self.arm_grass_pluck != mes_cad.arm_grass_pluck:
                    self.arm_grass_pluck = mes_cad.arm_grass_pluck
                    self.move_pluck(self.arm_grass_pluck)

                if self.arm_grass_wrist != mes_cad.arm_grass_wrist:
                    self.arm_grass_wrist = mes_cad.arm_grass_wrist
                    self.move_wrist(self.arm_grass_wrist)

                if self.arm_grass_elbow != mes_cad.arm_grass_elbow:
                    self.arm_grass_elbow = mes_cad.arm_grass_elbow
                    self.move_elbow(self.arm_grass_elbow)

                if self.arm_grass_sholder != mes_cad.arm_grass_sholder:
                    self.arm_grass_sholder = mes_cad.arm_grass_sholder
                    self.move_sholder(self.arm_grass_sholder)

                if self.arm_grass_base != mes_cad.arm_grass_base:
                    self.arm_grass_base = mes_cad.arm_grass_base
                    self.move_base(self.arm_grass_base)

                if self.arm_grass_lid != mes_cad.arm_grass_lid:
                    self.arm_grass_lid = mes_cad.arm_grass_lid
                    self.move_lid(self.arm_grass_lid)

                if self.arm_grass_splay != mes_cad.arm_grass_splay:
                    self.arm_grass_splay = mes_cad.arm_grass_splay
                    self.move_splay(self.arm_grass_splay)

                if self.arm_grass_blade != mes_cad.arm_grass_blade:
                    self.arm_grass_blade = mes_cad.arm_grass_blade
                    self.move_blade(self.arm_grass_blade)

            elif self.target_now == TARGET.SIDE_SPROUT:  # 芽かきモード時
                if self.arm_sprout_grub != mes_cad.arm_sprout_grub:
                    self.arm_sprout_grub = mes_cad.arm_sprout_grub
                    self.move_grub(self.arm_sprout_grub)

                if self.arm_sprout_twist != mes_cad.arm_sprout_twist:
                    self.arm_sprout_twist = mes_cad.arm_sprout_twist
                    self.move_twist(self.arm_sprout_twist)

                if self.arm_sprout_attach_r != mes_cad.arm_sprout_attach_r:
                    self.arm_sprout_attach_r = mes_cad.arm_sprout_attach_r
                    self.attach_r(self.arm_sprout_attach_r)

                if self.arm_sprout_attatch_l != mes_cad.arm_sprout_attatch_l:
                    self.arm_sprout_attatch_l = mes_cad.arm_sprout_attatch_l
                    self.attatch_l(self.arm_sprout_attatch_l)

                if self.arm_sprout_vertical != mes_cad.arm_sprout_vertical:
                    self.arm_sprout_vertical = mes_cad.arm_sprout_vertical
                    self.move_twistv(self.arm_sprout_vertical)

                if self.arm_sprout_horizontal != mes_cad.arm_sprout_horizontal:
                    self.arm_sprout_horizontal = mes_cad.arm_sprout_horizontal
                    self.move_twisth(self.arm_sprout_horizontal)

            elif self.target_now == TARGET.TOMATO:  # 収穫モード時

                if self.arm_crop_hand != mes_cad.arm_crop_hand:
                    self.arm_crop_hand = mes_cad.arm_crop_hand
                    self.move_hand(self.arm_crop_hand)

                if self.arm_crop_wrist != mes_cad.arm_crop_wrist:
                    self.arm_crop_wrist = mes_cad.arm_crop_wrist
                    self.move_wrist(self.arm_crop_wrist)

                if self.arm_crop_vertical != mes_cad.arm_crop_vertical:
                    self.arm_crop_vertical = mes_cad.arm_crop_vertical
                    self.move_handv(self.arm_crop_vertical)

                if self.arm_crop_horizontal != mes_cad.arm_crop_horizontal:
                    self.arm_crop_horizontal = mes_cad.arm_crop_horizontal
                    self.move_handh(self.arm_crop_horizontal)

            else:
                pass
        elif self.mode_now == MODE.MANUAL:
            pass
        else:
            pass
        
        print("TARGET=%s" % TARGET(self.target_now).name)
        print("arm_grass_hand=%s" % self.arm_grass_hand)
        print("arm_grass_pluck=%s" % self.arm_grass_pluck)
        print("arm_grass_wrist=%s" % self.arm_grass_wrist)
        print("arm_grass_elbow=%s" % self.arm_grass_elbow)
        print("arm_grass_sholder=%s" % self.arm_grass_sholder)
        print("arm_grass_lid=%s" % self.arm_grass_lid)
        print("arm_grass_splay=%s" % self.arm_grass_splay)
        print("arm_grass_blade=%s" % self.arm_grass_blade)
        print("arm_crop_hand=%s" % self.arm_crop_hand)
        print("arm_crop_wrist=%s" % self.arm_crop_wrist)
        print("arm_crop_vertical=%s" % self.arm_crop_vertical)
        print("arm_crop_horizontal=%s" % self.arm_crop_horizontal)
        print("arm_sprout_grub=%s" % self.arm_sprout_grub)
        print("arm_sprout_twist=%s" % self.arm_sprout_twist)
        print("arm_sprout_attach_r=%s" % self.arm_sprout_attach_r)
        print("arm_sprout_attatch_l=%s" % self.arm_sprout_attatch_l)
        print("arm_sprout_vertical=%s" % self.arm_sprout_vertical)
        print("arm_sprout_horizontal=%s" % self.arm_sprout_horizontal)

        print("============================================")

        # 区切り
        self.is_abh_call = False
    # end callback

    def mode_grass(self):
        """
        草刈り
        土台 => 肩 => ハンド => 引抜 => (肘) => 手首 => 散布ファン => ハンド => 引抜 =>
        => 肩 => 土台(66.7%) => 蓋 => 手首 => 肘 => ハンド => 蓋 => 刃
        """
        tmp_handx = self.mes_brain.handx_req
        tmp_handy = self.mes_brain.handy_req
        tmp_handz = self.mes_brain.handz_req

        # 土台モーターの角度計算
        # X座標とY座標で算出
        tmp_base = (math.atan2(tmp_handy, tmp_handx) / 90.0) * 100.0

        # 肩モーターの角度計算
        # Y座標とZ座標で算出
        tmp_should = (math.atan2(tmp_handz, tmp_handy) / 90.0) * 100.0

        # 手首モーターの角度計算
        tmp_wrist = 90.0 - tmp_should

        # モーター動作
        self.move_base(tmp_base)
        self.move_should(tmp_should)
        self.move_hand(RELEASE_HAND)
        self.move_pluck(ON_PLUCK)
        # self.move_elbow()
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
    # end mode_grass

    def mode_sprout(self):
        """
        芽かき
        ねじ切り垂直 => ねじ切り水平 => 添え手右・添え手左 => 枝掴み =>
        => 枝ねじり => 枝掴み => 添え手右・添え手左 => ねじ切り水平
        """
        tmp_twistx = self.mes_brain.twistx_req
        tmp_twistz = self.mes_brain.twistz_req

        tmp_twistv = tmp_twistz / STEP_1PULSE
        tmp_twisth = tmp_twistx / STEP_1PULSE

        self.move_twistv(tmp_twistv)
        self.move_twisth(tmp_twisth)
        self.move_attach_l(LIM_ATTACH_LR)
        self.move_attach_r(LIM_ATTACH_RL)
        self.move_grab(CATCH_GRAB)
        self.move_twist(ON_TWIST)
        self.move_grab(RELEASE_GRAB)
        self.move_attach_l(LIM_ATTACH_LL)
        self.move_attach_r(LIM_ATTACH_RR)
        self.move_twisth(LIM_TWISTH_MIN)
    # end mode_sprout

    def mode_tomato(self):
        """
        収穫
        ハンド垂直 => ハンド水平 => ハンド => 手首
         =>ハンド => ハンド水平 => 手首
        """
        handx = self.mes_brain.handx_req
        handz = self.mes_brain.handz_req

        handv = handz / STEP_1PULSE
        handh = handx / STEP_1PULSE

        self.move_handv(handv)  # ハンド垂直
        self.move_handh(handh)  # ハンド水平
        self.move_hand(CATCH_HAND)  # ハンド
        self.move_wrist(LIM_WRIST_F)  # 手首
        self.move_hand(RELEASE_HAND)  # ハンド
        self.move_handh(handh)  # ハンド水平
        self.move_wrist(LIM_WRIST_B)  # 手首
        self.move_handh(LIM_HANDH_MIN)  # ハンド水平
    # end mode_tomato


def abh_py():
    """
    アーム・ボディ・ハンドのメイン
    """
    abhc = AbhClass()

    rospy.init_node('abh_py_node', anonymous=True)
    rrate = rospy.Rate(CYCLES)
    rospy.Subscriber('brain', brain, abhc.callback, queue_size=1)
    rospy.Subscriber('client_abh_debug', client_abh_debug,
                     abhc.callback_debug, queue_size=1)
    print("start_abh")
    # ctl + Cで終了しない限りwhileループでpublishし続ける
    while not rospy.is_shutdown():
        abhc.publish_data()
        rrate.sleep()


# end abh_py
if __name__ == '__main__':
    abh_py()
