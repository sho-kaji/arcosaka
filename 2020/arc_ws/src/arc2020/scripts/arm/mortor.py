#!/usr/bin/env python
# -*- coding: utf-8 -*-
# pylint: disable=E1101,C0325
"""
モーター制御
"""

import time
import atexit
import os

from mortor_consts import \
    ADDR_PWM, \
    STEPROTATE, STEP_1PULSE, STEP_FREQ, STEP_DUTY, \
    STEP_MIN, STEP_MAX, \
    SERVO_MIN_K, SERVO_MAX_K, \
    SERVO_FREQ, SERVO_MIN, SERVO_MAX, \
    SRV0MAX_K, SRV0MIN_K

if os.name == 'posix':
    import pigpio
    #import pca9685
    # Import the PCA9685 module.
    import Adafruit_PCA9685

RET_ORGSW = 15

class ServoMortorClass(object):
    """
    サーボモータークラス
    """

    def __init__(self, is_debug=False):
        self.is_notdebug = not((os.name != 'posix') or is_debug)

        if self.is_notdebug:
            try:
                # initialize move_servo
                self.pwm = Adafruit_PCA9685.PCA9685(ADDR_PWM)
                #self.pwm = pca9685.Pca9685Class(ADDR_PWM)
                self.pwm.set_pwm_freq(SERVO_FREQ)

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

    def posinit(self, channel):
        """
        位置初期化
        """
        lim_srt = SERVO_MIN

        print("servo mortor init c=%d\tsrt=%d" % (channel, lim_srt))
        self.move_servo_pulse(channel, lim_srt)
    # end posinit

    def move_servo(self, channel, power):
        """
        サーボモーター
        """

        lim_max = SRV0MAX_K
        lim_min = SRV0MIN_K

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

    def endfnc(self):
        """
        終了処理
        """
        if self.is_notdebug:
            self.pwm.set_all_pwm(0, 4096)  # 全モーターPWM解除
    # end endfnc

# end ServoMortorClass



class StepMortorClass(object):
    """
    ステップモータークラス
    """

    def __init__(self, is_debug=False, ports=(16, 20), limit=(STEP_MIN, STEP_MAX), port_en=18):
        self.is_notdebug = not((os.name != 'posix') or is_debug)

        if self.is_notdebug:
            try:
                # initialize gpio
                self.pic = pigpio.pi()

                self.port_a = ports[0]
                self.port_b = ports[1]
                print("step mortor port = %d,%d" % (self.port_a, self.port_b))
                self.port_en = port_en
                self.pic.set_mode(self.port_a, pigpio.OUTPUT)
                self.pic.set_mode(self.port_b, pigpio.OUTPUT)
                self.pic.set_mode(self.port_en, pigpio.OUTPUT)
                self.pic.set_mode(RET_ORGSW, pigpio.INPUT)

                if limit[0] < limit[1]:
                    self.limit_min = limit[0]
                    self.limit_max = limit[1]
                else:
                    self.limit_min = limit[1]
                    self.limit_max = limit[0]

                print("step mortor limit = %d,%d" %
                      (self.limit_min, self.limit_max))
                self.stepcnt = 0
                self.step_o = 0
                self.step_n = 0
                self.issetpos = False

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

    def posinit(self, steplotate):
        """
        モーター位置初期化
        """
        
        if steplotate == STEPROTATE.PLUS:
            limit = self.limit_max
        elif steplotate == STEPROTATE.MINUS:
            limit = self.limit_min
        else:
            limit = 0
        setpos = int(limit * steplotate)
        print("step mortor setpos = %d" % (setpos))
        self.issetpos = True
        self.move_posinit_step(setpos)
        self.issetpos = False
        self.stepcnt = 0
    # end posinit

    def move_posinit_step(self, freq=-1):

        if freq < 0:
            freq = STEP_FREQ


        wait_hl = (1.0 / freq * (STEP_DUTY / 100.0))
        wait_lh = (1.0 / freq * (1 - (STEP_DUTY / 100.0)))

        if self.is_notdebug:
            self.pic.set_mode(self.port_en, pigpio.OUTPUT)
            self.pic.set_mode(self.port_a, pigpio.OUTPUT)
            self.pic.set_mode(self.port_b, pigpio.OUTPUT)
            self.pic.set_mode(RET_ORGSW, pigpio.INPUT)
            # ENABLE端子
            self.pic.write(self.port_en, pigpio.HIGH)
            self.retorgsw = self.pic.read(RET_ORGSW)
            
            while self.retorgsw == 0:
                self.pic.write(self.port_a, pigpio.HIGH)
                time.sleep(wait_hl/2)
                self.pic.write(self.port_b, pigpio.HIGH)
                time.sleep(wait_hl/2)
                self.pic.write(self.port_a, pigpio.LOW)
                time.sleep(wait_lh/2)
                self.pic.write(self.port_b, pigpio.LOW)
                time.sleep(wait_lh/2)
                
                self.retorgsw = self.pic.read(RET_ORGSW)

        self.endfnc()

    # end move_posinit_step

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

        self.step_n = step - self.step_o
        self.step_o = step
        stepping = self.stepcnt + self.step_n
        print("step mortor     step = %d" % (self.step_n))
        print("step mortor stepping = %d" % (stepping))

        wait_hl = (1.0 / freq * (STEP_DUTY / 100.0))
        wait_lh = (1.0 / freq * (1 - (STEP_DUTY / 100.0)))

        if self.is_notdebug:
            self.pic.set_mode(self.port_en, pigpio.OUTPUT)
            self.pic.set_mode(self.port_a, pigpio.OUTPUT)
            self.pic.set_mode(self.port_b, pigpio.OUTPUT)
            # ENABLE端子
            if self.step_n != 0 :
                self.pic.write(self.port_en, pigpio.HIGH)

        for i in range(abs(self.step_n)):
            if self.issetpos or (\
                    self.is_notdebug \
                    and (self.limit_min <= self.stepcnt) \
                    and (self.stepcnt <= self.limit_max
                )
            ):
                if stepping > self.stepcnt:
                    self.pic.write(self.port_a, pigpio.HIGH)
                    time.sleep(wait_hl/2)
                    self.pic.write(self.port_b, pigpio.HIGH)
                    time.sleep(wait_hl/2)
                    self.pic.write(self.port_a, pigpio.LOW)
                    time.sleep(wait_lh/2)
                    self.pic.write(self.port_b, pigpio.LOW)
                    time.sleep(wait_lh/2)
                    self.stepcnt += 1

                elif stepping < self.stepcnt:
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
                if stepping > self.stepcnt:
                    print("port_a ON  %f" % wait_hl)
                    time.sleep(wait_hl)
                    print("port_b ON  %f" % wait_hl)
                    time.sleep(wait_hl)
                    print("port_a OFF %f" % wait_lh)
                    time.sleep(wait_lh)
                    print("port_b OFF %f" % wait_lh)
                    time.sleep(wait_lh)
                    self.stepcnt += 1

                elif stepping < self.stepcnt:
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
            #print("pulse  now = %d\tpulse goto = %d" %
            #      (self.stepcnt, step))
            if ((self.stepcnt <= self.limit_min) \
                or (self.limit_max <= self.stepcnt)) \
                and not(self.issetpos):
                break
            #print("pulse     = %d/%d" % (i+1, step))
        # end for i in range(abs(step))

        self.endfnc()

    # end move_step_step
# end StepMortorClass