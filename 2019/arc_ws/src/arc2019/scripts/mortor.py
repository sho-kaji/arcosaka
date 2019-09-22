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

    def __init__(self):
        # initialize gpio
        self.pic = pigpio.pi()

        # initialize move_dc
        self.dcduty_cw_o = 0
        self.dcduty_ccw_o = 0

        # initialize move_servo
        self.pwm = Adafruit_PCA9685.PCA9685(ADDR_PWM)
        self.pwm.set_pwm_freq(60)


    def endfnc(self):
        """
        終了処理
        """
        self.pwm.set_all_pwm(0, 0) # 全モーターPWM解除


    def move_dc(self, port_cw, port_ccw, rotate, limit=True):
        """
        DCモーター
        """

        #とりあえず初期化
        dcduty_cw = 0
        dcduty_ccw = 0

        #Duty計算
        if rotate != DCROTATE.STOP:
            dcduty_cw = self.dcduty_cw_o + (DC_PLUS * rotate * 1000)
            dcduty_ccw = self.dcduty_ccw_o - (DC_PLUS * rotate * 1000)
        else:
            dcduty_cw = 0
            dcduty_ccw = 0

        #上下限ガード
        if limit:
            if dcduty_cw > DC_DUTY:
                dcduty_cw = DC_DUTY

        if dcduty_cw > 1000000: #ライブラリの上限値
            dcduty_cw = 1000000

        if dcduty_cw < 0:
            dcduty_cw = 0

        if limit:
            if dcduty_ccw > DC_DUTY:
                dcduty_ccw = DC_DUTY

        if dcduty_ccw > 1000000: #ライブラリの上限値
            dcduty_ccw = 1000000

        if dcduty_ccw < 0:
            dcduty_ccw = 0

        print("cw  = %d" % dcduty_cw)
        print("ccw = %d" % dcduty_ccw)

        #新しい値で出力
        self.pic.set_PWM_frequency(port_cw, DC_FREQ)
        self.pic.set_PWM_frequency(port_ccw, DC_FREQ)

        self.pic.set_PWM_dutycycle(port_cw, dcduty_cw)
        self.pic.set_PWM_dutycycle(port_ccw, dcduty_ccw)

        #今回値を保存
        self.dcduty_cw_o = dcduty_cw
        self.dcduty_ccw_o = dcduty_ccw


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
