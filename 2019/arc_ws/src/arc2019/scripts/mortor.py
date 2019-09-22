#!/usr/bin/python
# -*- coding: utf-8 -*-
"""
モーター制御
"""

import time
import pigpio

# Import the PCA9685 module.
import Adafruit_PCA9685

from mortor_consts import \
    ADDR_PWM, \
    DCROTATE, \
    DC_FREQ, \
    DC_DUTY, \
    DC_PLUS, \
    SERVO_MIN, \
    SERVO_MAX, \
    STEP_1PULSE, \
    STEP_FREQ, \
    STEP_DUTY

class MortorClass(object):
    """
    モータークラス
    """

    def __init__(self, port_a_cw=16, port_a_ccw=20, port_b_cw=19, port_b_ccw=26):
        # initialize gpio
        self.pic = pigpio.pi()

        self.port_a_cw = port_a_cw
        self.port_a_ccw = port_a_ccw
        self.port_b_cw = port_b_cw
        self.port_b_ccw = port_b_ccw

        # initialize move_dc
        self.dcduty_a_cw_o = 0
        self.dcduty_a_ccw_o = 0
        self.dcduty_b_cw_o = 0
        self.dcduty_b_ccw_o = 0

        # initialize move_servo
        self.pwm = Adafruit_PCA9685.PCA9685(ADDR_PWM)
        self.pwm.set_pwm_freq(60)


    def endfnc(self):
        """
        終了処理
        """
        self.pwm.set_all_pwm(0, 0) # 全モーターPWM解除

    def move_dc(self, rotate_a, rotate_b, limit=True):
        """
        DCモーター
        """

        #とりあえず初期化
        dcduty_a_cw = 0
        dcduty_a_ccw = 0
        dcduty_b_cw = 0
        dcduty_b_ccw = 0

        while True:
            #Duty計算
            if rotate_a != DCROTATE.STOP:
                dcduty_a_cw = self.dcduty_a_cw_o + (DC_PLUS * rotate_a * 1000)
                dcduty_a_ccw = self.dcduty_a_ccw_o - (DC_PLUS * rotate_a * 1000)
            else:
                dcduty_a_cw = 0
                dcduty_a_ccw = 0

            if rotate_b != DCROTATE.STOP:
                dcduty_b_cw = self.dcduty_b_cw_o + (DC_PLUS * rotate_b * 1000)
                dcduty_b_ccw = self.dcduty_b_ccw_o - (DC_PLUS * rotate_b * 1000)
            else:
                dcduty_b_cw = 0
                dcduty_b_ccw = 0

            #上下限ガード
            if limit:
                if dcduty_a_cw > DC_DUTY:
                    dcduty_a_cw = DC_DUTY

                if dcduty_a_ccw > DC_DUTY:
                    dcduty_a_ccw = DC_DUTY

                if dcduty_b_cw > DC_DUTY:
                    dcduty_b_cw = DC_DUTY

                if dcduty_b_ccw > DC_DUTY:
                    dcduty_b_ccw = DC_DUTY

            if dcduty_a_cw > 100: #ライブラリの上限値
                dcduty_a_cw = 100
            if dcduty_a_cw < 0:
                dcduty_a_cw = 0
            if dcduty_a_ccw > 100: #ライブラリの上限値
                dcduty_a_ccw = 100
            if dcduty_a_ccw < 0:
                dcduty_a_ccw = 0

            # 前回値と同じなら処理を抜ける
            if dcduty_a_cw != self.dcduty_a_cw_o \
            and dcduty_a_ccw != self.dcduty_a_ccw_o \
            and dcduty_b_cw != self.dcduty_b_cw_o \
            and dcduty_b_ccw != self.dcduty_b_ccw_o:
                break

            self.move_dc_duty(self.port_a_cw, self.port_a_ccw, dcduty_a_cw, dcduty_a_ccw)
            self.move_dc_duty(self.port_b_cw, self.port_b_ccw, dcduty_b_cw, dcduty_b_ccw)

            #今回値を保存
            self.dcduty_a_cw_o = dcduty_a_cw
            self.dcduty_a_ccw_o = dcduty_a_ccw
            self.dcduty_b_cw_o = dcduty_b_cw
            self.dcduty_b_ccw_o = dcduty_b_ccw

            time.sleep(1)

    def move_dc_duty(self, port_cw, port_ccw, dcduty_cw, dcduty_ccw):
        """
        DCモーターデューティー
        """
        #上下限ガード
        if dcduty_cw > 100: #ライブラリの上限値
            dcduty_cw = 100
        if dcduty_cw < 0:
            dcduty_cw = 0
        if dcduty_ccw > 100: #ライブラリの上限値
            dcduty_ccw = 100
        if dcduty_ccw < 0:
            dcduty_ccw = 0

        if dcduty_cw != 0 and dcduty_ccw != 0:
            dcduty_cw = 0
            dcduty_ccw = 0

        print("cw  = %d" % dcduty_cw)
        print("ccw = %d" % dcduty_ccw)

        #新しい値で出力
        self.pic.set_PWM_frequency(port_cw, DC_FREQ)
        self.pic.set_PWM_frequency(port_ccw, DC_FREQ)

        self.pic.set_PWM_range(port_cw, 100)
        self.pic.set_PWM_range(port_ccw, 100)

        self.pic.set_PWM_dutycycle(port_cw, dcduty_cw)
        self.pic.set_PWM_dutycycle(port_ccw, dcduty_ccw)


    def move_servo(self, channel, power, limit=True):
        """
        サーボモーター
        """
        if power < 0:
            power = 0
        if power > 100:
            power = 100

        pulse = ((SERVO_MAX - SERVO_MIN) * (power / 100.0)) + SERVO_MIN
        self.move_servo_pulse(channel, pulse, limit)

    def move_servo_pulse(self, channel, pulse, limit=True):
        """
        サーボモーターパルス
        """
        print("pulse = %d" % pulse)

        if limit:
            if pulse < SERVO_MIN:
                pulse = SERVO_MIN
            if pulse > SERVO_MAX:
                pulse = SERVO_MAX

        self.pwm.set_pwm(channel, 0, int(pulse))

    def move_step(self, port_a, port_b, distance):
        """
        ステッピングモーター
        """

        # 周波数とDuty比から1パルスの待機時間を計算
        step = distance / STEP_1PULSE
        wait_hl = (STEP_FREQ * (STEP_DUTY / 100.0)) / 2
        wait_lh = (STEP_FREQ / 2) - wait_hl

        for i in range(abs(step)):
            if step > 0:
                self.pic.write(port_a, pigpio.HIGH)
                time.sleep(wait_hl)
                self.pic.write(port_b, pigpio.HIGH)
                time.sleep(wait_hl)
                self.pic.write(port_a, pigpio.LOW)
                time.sleep(wait_lh)
                self.pic.write(port_b, pigpio.LOW)
                time.sleep(wait_lh)

            elif step < 0:
                self.pic.write(port_b, pigpio.HIGH)
                time.sleep(wait_hl)
                self.pic.write(port_a, pigpio.HIGH)
                time.sleep(wait_hl)
                self.pic.write(port_b, pigpio.LOW)
                time.sleep(wait_lh)
                self.pic.write(port_a, pigpio.LOW)
                time.sleep(wait_lh)
