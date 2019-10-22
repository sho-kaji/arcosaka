#!/usr/bin/env python
# -*- coding: utf-8 -*-
# pylint: disable=E1101,C0325
"""
モーター制御
"""

import time
import atexit
import os

from params import TARGET
from mortor_consts import \
    ADDR_PWM, \
    DCROTATE, \
    DC_FREQ, DC_DUTY, DC_PLUS, \
    SERVO_MIN_K, SERVO_MAX_K, \
    SERVO_MIN_M, SERVO_MAX_M, \
    SERVO_FREQ, SERVO_MIN, SERVO_MAX, \
    STEPROTATE, STEP_1PULSE, STEP_FREQ, STEP_DUTY

if os.name == 'posix':
    import pigpio
    # Import the PCA9685 module.
    import Adafruit_PCA9685


class DcMortorClass(object):
    """
    DCモータークラス
    """

    def __init__(self, is_debug=False, ports=(16, 20, 19, 26), limit=DC_DUTY):
        self.is_notdebug = not((os.name != 'posix') or is_debug)

        if self.is_notdebug:
            try:
                # initialize gpio
                self.pic = pigpio.pi()

                self.port_a_cw = ports[0]
                self.port_a_ccw = ports[1]
                if len(ports) > 2:
                    self.port_b_cw = ports[2]
                    self.port_b_ccw = ports[3]
                else:
                    self.port_b_cw = -1
                    self.port_b_ccw = -1

                self.limit_min = -limit
                self.limit_max = limit

                # initialize move_dc
                self.dcduty_a_o = 0
                self.dcduty_b_o = 0

            except TypeError as ex:
                print(ex)
                self.is_notdebug = False

        else:
            pass

        if self.is_notdebug:
            print("DC mortor is move")
        else:
            print("DC mortor is debug")
    # end __init__

    def endfnc(self):
        """
        終了処理
        """
        if self.is_notdebug:
            self.move_dc_duty(self.port_a_cw, self.port_a_ccw, 0, 0)
            self.move_dc_duty(self.port_b_cw, self.port_b_ccw, 0, 0)
    # end endfnc

    def set_debug(self, val):
        """
        デバッグモード設定
        """
        self.is_notdebug = not(val)
    # end set_debug

    def move_dc(self, rotate_a, rotate_b, limit=True):
        """
        DCモーター(なまし付き)
        """

        # とりあえず初期化
        dcduty_a = 0
        dcduty_b = 0

        while True:
            # Duty計算
            if rotate_a != DCROTATE.STOP:
                dcduty_a = self.dcduty_a_o + (DC_PLUS * rotate_a)
            elif self.dcduty_a_o > 0:
                dcduty_a = self.dcduty_a_o - DC_PLUS
            elif self.dcduty_a_o < 0:
                dcduty_a = self.dcduty_a_o + DC_PLUS

            if rotate_b != DCROTATE.STOP:
                dcduty_b = self.dcduty_b_o + (DC_PLUS * rotate_b)
            elif self.dcduty_b_o > 0:
                dcduty_b = self.dcduty_b_o - DC_PLUS
            elif self.dcduty_b_o < 0:
                dcduty_b = self.dcduty_b_o + DC_PLUS

            # 上下限ガード
            if limit:
                if dcduty_a > self.limit_max:
                    dcduty_a = self.limit_max
                if dcduty_a < self.limit_min:
                    dcduty_a = self.limit_min
                if dcduty_b > self.limit_max:
                    dcduty_b = self.limit_max
                if dcduty_b < self.limit_min:
                    dcduty_b = self.limit_min

            if dcduty_a > 100:
                dcduty_a = 100
            if dcduty_a < -100:
                dcduty_a = -100

            if dcduty_b > 100:
                dcduty_b = 100
            if dcduty_b < -100:
                dcduty_b = -100

            # 前回値と同じなら処理を抜ける
            if dcduty_a == self.dcduty_a_o and dcduty_b == self.dcduty_b_o:
                break

            dcduty_a_cw = 0
            dcduty_a_ccw = 0
            if dcduty_a > 0:
                dcduty_a_cw = dcduty_a
            elif dcduty_a < 0:
                dcduty_a_ccw = dcduty_a

            dcduty_b_cw = 0
            dcduty_b_ccw = 0
            if dcduty_b > 0:
                dcduty_b_cw = dcduty_b
            elif dcduty_b < 0:
                dcduty_b_ccw = dcduty_b

            #print("cw  A = %d" % dcduty_a_cw)
            #print("ccw A = %d" % dcduty_a_ccw)

            if self.port_a_cw >= 0 and self.port_a_ccw >= 0:
                self.move_dc_duty(
                    self.port_a_cw, self.port_a_ccw, dcduty_a_cw, dcduty_a_ccw)

            #print("cw  B = %d" % dcduty_b_cw)
            #print("ccw B = %d" % dcduty_b_ccw)

            if self.port_b_cw >= 0 and self.port_b_ccw >= 0:
                self.move_dc_duty(
                    self.port_b_cw, self.port_b_ccw, dcduty_b_cw, dcduty_b_ccw)

            # 今回値を保存
            self.dcduty_a_o = dcduty_a
            self.dcduty_b_o = dcduty_b

            time.sleep(0.05)
    # end move_dc

    def move_dc_duty(self, port_cw, port_ccw, dcduty_cw, dcduty_ccw):
        """
        DCモーターデューティー
        """
        if (port_cw < 0) or (port_ccw < 0):
            return
        # 絶対値に変換
        dcduty_cw = abs(dcduty_cw)
        dcduty_ccw = abs(dcduty_ccw)

        # 上下限ガード
        if dcduty_cw > 100:
            dcduty_cw = 100
        if dcduty_cw < 0:
            dcduty_cw = 0
        if dcduty_ccw > 100:
            dcduty_ccw = 100
        if dcduty_ccw < 0:
            dcduty_ccw = 0

        if dcduty_cw != 0 and dcduty_ccw != 0:
            dcduty_cw = 0
            dcduty_ccw = 0

        if self.is_notdebug:
            # 新しい値で出力
            self.pic.set_PWM_frequency(port_cw, DC_FREQ)
            self.pic.set_PWM_frequency(port_ccw, DC_FREQ)

            self.pic.set_PWM_range(port_cw, 100)
            self.pic.set_PWM_range(port_ccw, 100)

            self.pic.set_PWM_dutycycle(port_cw, dcduty_cw)
            self.pic.set_PWM_dutycycle(port_ccw, dcduty_ccw)
        else:
            print ("dcduty_cw=%3d\tcduty_ccw=%3d" % (dcduty_cw, dcduty_ccw))
    # end move_dc_duty
# end DcMortorClass


class ServoMortorClass(object):
    """
    サーボモータークラス
    """

    def endfnc(self):
        """
        終了処理
        """
        if self.is_notdebug:
            self.pwm.set_all_pwm(0, 4096)  # 全モーターPWM解除
    # end endfnc

    def __init__(self, is_debug=False, target=TARGET.UNKNOWN):
        self.is_notdebug = not((os.name != 'posix') or is_debug)

        if self.is_notdebug:
            try:
                # initialize move_servo
                self.pwm = Adafruit_PCA9685.PCA9685(ADDR_PWM)
                self.pwm.set_pwm_freq(SERVO_FREQ)
                self.target = target

            except TypeError as ex:
                print(ex)
                self.is_notdebug = False

        else:
            pass

        atexit.register(self.endfnc)

        if self.is_notdebug:
            print("servo mortor is move")
        else:
            print("servo mortor is debug")
    # end __init__

    def move_servo(self, channel, power, lim_min, lim_max):
        """
        サーボモーター
        """

        if lim_min > lim_max:
            num_tmp = lim_max
            lim_max = lim_min
            lim_min = num_tmp

        # 上下限ガード
        if power < 0:
            power = 0
        if power > 100:
            power = 100

        # 割合からパルスを計算
        pulse = ((lim_max - lim_min) * (power / 100.0)) + lim_min
        self.move_servo_pulse(channel, pulse)
    # end move_servo

    def move_servo_pulse(self, channel, pulse):
        """
        サーボモーターパルス
        """

        if pulse > SERVO_MAX:
            pulse = SERVO_MAX
        elif pulse < SERVO_MIN:
            pulse = SERVO_MIN
        else:
            pass

        print("pulse = %d" % pulse)

        if self.is_notdebug:
            self.pwm.set_pwm(channel, 0, int(pulse))
    # end move_servo_pulse
# end ServoMortorClass


class StepMortorClass(object):
    """
    ステップモータークラス
    """

    def __init__(self, is_debug=False, ports=(16, 20), limit=(0, 3500), port_en=18):
        self.is_notdebug = not((os.name != 'posix') or is_debug)

        if self.is_notdebug:
            try:
                # initialize gpio
                self.pic = pigpio.pi()

                self.port_a = ports[0]
                self.port_b = ports[1]
                self.port_en = port_en

                if limit[0] < limit[1]:
                    self.limit_min = limit[0]
                    self.limit_max = limit[1]
                else:
                    self.limit_min = limit[1]
                    self.limit_max = limit[0]
                self.stepcnt = 0

            except TypeError as ex:
                print(ex)
                self.is_notdebug = False

        else:
            pass

        if self.is_notdebug:
            print("step mortor is move")
        else:
            print("step mortor is debug")
    # end __init__

    def endfnc(self):
        """
        終了処理
        """
        if self.stepcnt < self.limit_min:
            self.stepcnt = self.limit_min
        elif self.limit_max < self.stepcnt:
            self.stepcnt = self.limit_max
        else:
            pass
        if self.is_notdebug:
            self.pic.write(self.port_en, pigpio.LOW)
            self.pic.write(self.port_a, pigpio.LOW)
            self.pic.write(self.port_b, pigpio.LOW)
    # end endfnc

    def resetpos(self, steplotate):
        """
        モーター位置初期化
        """
        if steplotate != STEPROTATE.STOP:
            self.move_step_step(self.limit_max * steplotate)
        else:
            pass
        self.stepcnt = 0
    # end resetpos

    def move_step(self, distance):
        """
        ステッピングモーター
        """
        # 要求値を1ステップ当たりの距離で割る
        step = int(distance / STEP_1PULSE)
        self.move_step_step(step)
    # end move_step

    def move_step_step(self, step, freq=-1):
        """
        ステッピングモーターステップ数入力
        """
        if freq < 0:
            freq = STEP_FREQ

        wait_hl = (1.0 / freq * (STEP_DUTY / 100.0))
        wait_lh = (1.0 / freq * (1 - (STEP_DUTY / 100.0)))

        if self.is_notdebug:
            self.pic.set_mode(self.port_en, pigpio.OUTPUT)
            self.pic.set_mode(self.port_a, pigpio.OUTPUT)
            self.pic.set_mode(self.port_b, pigpio.OUTPUT)
            # ENABLE端子
            self.pic.write(self.port_en, pigpio.HIGH)

        for i in range(abs(step)):
            if self.is_notdebug \
                    and (self.limit_min <= self.stepcnt) \
                    and (self.stepcnt <= self.limit_max):
                if step > 0:
                    self.pic.write(self.port_a, pigpio.HIGH)
                    time.sleep(wait_hl/2)
                    self.pic.write(self.port_b, pigpio.HIGH)
                    time.sleep(wait_hl/2)
                    self.pic.write(self.port_a, pigpio.LOW)
                    time.sleep(wait_lh/2)
                    self.pic.write(self.port_b, pigpio.LOW)
                    time.sleep(wait_lh/2)
                    self.stepcnt += 1

                elif step < 0:
                    self.pic.write(self.port_b, pigpio.HIGH)
                    time.sleep(wait_hl/2)
                    self.pic.write(self.port_a, pigpio.HIGH)
                    time.sleep(wait_hl/2)
                    self.pic.write(self.port_b, pigpio.LOW)
                    time.sleep(wait_lh/2)
                    self.pic.write(self.port_a, pigpio.LOW)
                    time.sleep(wait_lh/2)
                    self.stepcnt -= 1
            elif not(self.is_notdebug):
                if step > 0:
                    print("port_a ON  %f" % wait_hl)
                    time.sleep(wait_hl)
                    print("port_b ON  %f" % wait_hl)
                    time.sleep(wait_hl)
                    print("port_a OFF %f" % wait_lh)
                    time.sleep(wait_lh)
                    print("port_b OFF %f" % wait_lh)
                    time.sleep(wait_lh)
                    self.stepcnt += 1

                elif step < 0:
                    print("port_b ON  %f" % wait_hl)
                    time.sleep(wait_hl)
                    print("port_a ON  %f" % wait_hl)
                    time.sleep(wait_hl)
                    print("port_b OFF %f" % wait_lh)
                    time.sleep(wait_lh)
                    print("port_a OFF %f" % wait_lh)
                    time.sleep(wait_lh)
                    self.stepcnt -= 1
            else:
                break
            # end if self.is_notdebug
            print("pulse     = %d/%d" % (self.stepcnt, step))
            if (self.stepcnt <= self.limit_min) or (self.limit_max <= self.stepcnt):
                break
            #print("pulse     = %d/%d" % (i+1, step))
        # end for i in range(abs(step))

        self.endfnc()

    # end move_step_step
# end StepMortorClass
