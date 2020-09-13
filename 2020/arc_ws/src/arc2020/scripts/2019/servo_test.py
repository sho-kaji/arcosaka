#!/usr/bin/env python
# -*- coding: utf-8 -*-
# pylint: disable=E1101,C0325
"""
サーボモーターテスト
"""
import atexit
import os

from params import TARGET
from mortor_consts import \
    ADDR_PWM, \
    SERVO_FREQ, SERVO_MIN, SERVO_MAX

if os.name == 'posix':
    # Import the PCA9685 module.
    import Adafruit_PCA9685

class ServoTest(object):
    """
    サーボモータークラス
    """

    def endfnc(self):
        """
        終了処理
        """
        self.pwm.set_all_pwm(4096, 4096)
    #end endfnc

    def __init__(self):
        try:
            # initialize move_servo
            self.pwm = Adafruit_PCA9685.PCA9685(ADDR_PWM)
            self.pwm.set_pwm_freq(SERVO_FREQ)

        except TypeError as ex:
            print(ex)

        atexit.register(self.endfnc)

    #end __init__

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

        self.pwm.set_pwm(channel, 0, int(pulse))
    #end move_servo_pulse
#end ServoMortorClass
