#!/usr/bin/env python
# -*- coding: utf-8 -*-
# pylint: disable=E1101,C0325
"""
サーボモータードライバー
"""
import time
import math
import atexit
import pigpio

ADDR_PWM = 0x41

MODE1 = 0x00
MODE2 = 0x01
SUBADR1 = 0x02
SUBADR2 = 0x03
SUBADR3 = 0x04
PRESCALE = 0xFE
LED0_ON_L = 0x06
LED0_ON_H = 0x07
LED0_OFF_L = 0x08
LED0_OFF_H = 0x09
ALL_LED_ON_L = 0xFA
ALL_LED_ON_H = 0xFB
ALL_LED_OFF_L = 0xFC
ALL_LED_OFF_H = 0xFD

# Bits:
RESTART = 0x80
SLEEP = 0x10
ALLCALL = 0x01
INVRT = 0x10
OUTDRV = 0x04

class Pca9685Class(object):
    """
    サーボモータードライバークラス
    """

    def __init__(self, addr=ADDR_PWM):
        self.is_enable = False
        try:
            self.i2cpi = pigpio.pi()
            self.i2ch = self.i2cpi.i2c_open(1, addr)
            self.is_enable = True
        except IOError  as ex:
            print(ex)
            self.is_enable = False

        finally:
            atexit.register(self.endfnc)
    # end __init__

    def set_pwm_freq(self, freq_hz):
        """
        周波数を指定[Hz]
        """
        try:
            prescaleval = 25000000.0    # 25MHz
            prescaleval /= 4096.0       # 12-bit
            prescaleval /= float(freq_hz)
            prescaleval -= 1.0
            prescale = int(math.floor(prescaleval + 0.5))
            oldmode = self.i2cpi.i2c_read_byte_data(self.i2ch, MODE1)
            newmode = (oldmode & 0x7F) | 0x10    # sleep
            self.i2cpi.i2c_write_byte_data(self.i2ch, MODE1, newmode)
            self.i2cpi.i2c_write_byte_data(self.i2ch, PRESCALE, prescale)
            self.i2cpi.i2c_write_byte_data(self.i2ch, MODE1, oldmode)
            time.sleep(0.005)
            self.i2cpi.i2c_write_byte_data(self.i2ch, MODE1, oldmode | 0x80)
        except IOError  as ex:
            print(ex)
            self.is_enable = False
    # end set_pwm_freq

    def set_pwm(self, channel, pwm_on, pwm_off):
        """
        指定したモーターのパルスを指定
        """
        try:
            self.i2cpi.i2c_write_byte_data(self.i2ch, LED0_ON_L+4*channel, pwm_on & 0xFF)
            self.i2cpi.i2c_write_byte_data(self.i2ch, LED0_ON_H+4*channel, pwm_on >> 8)
            self.i2cpi.i2c_write_byte_data(self.i2ch, LED0_OFF_L+4*channel, pwm_off & 0xFF)
            self.i2cpi.i2c_write_byte_data(self.i2ch, LED0_OFF_H+4*channel, pwm_off >> 8)
        except IOError  as ex:
            print(ex)
            self.is_enable = False
    #end set_pwm

    def set_all_pwm(self, pwm_on, pwm_off):
        """
        すべてのモーターのパルスを指定
        """
        try:
            self.i2cpi.i2c_write_byte_data(self.i2ch, ALL_LED_ON_L, pwm_on & 0xFF)
            self.i2cpi.i2c_write_byte_data(self.i2ch, ALL_LED_ON_H, pwm_on >> 8)
            self.i2cpi.i2c_write_byte_data(self.i2ch, ALL_LED_OFF_L, pwm_off & 0xFF)
            self.i2cpi.i2c_write_byte_data(self.i2ch, ALL_LED_OFF_H, pwm_off >> 8)
        except IOError  as ex:
            print(ex)
            self.is_enable = False
    # end set_all_pwm

    def software_reset(self):
        """
        ソフトリセット
        """
        i2cpi = pigpio.pi()
        i2ch = i2cpi.i2c_open(1, 0x00)
        i2cpi.i2c_write_byte(i2ch, 0x06)  # SWRST
        self.set_all_pwm(0, 0)
        print("software_reset")
    # end software_reset

    def endfnc(self):
        """
        終了処理
        """
        self.software_reset()
        self.is_enable = False
    # end endfnc
# end Pca9685Class


def test():
    """
    テストモジュール
    """
    pc = Pca9685Class()
    pc.set_pwm_freq(60)

    while(pc.is_enable):
        try:
            mortornum = input('mortor:')
            int_port = int(mortornum)
            if int_port < 0:
                break

            val = input(' pulse:')
            int_tmp = int(val)
            if int_tmp < 0:
                break

            if int_port < 7:
                pc.set_pwm(int_port, 0, int(int_tmp))
            else:
                pc.set_all_pwm(0, int(int_tmp))

        except KeyboardInterrupt:
            print("Ctrl+Cで停止しました")
            break
        except TypeError as ex:
            print(ex)
    # end while(pc.is_enable)
# end test


if __name__ == '__main__':
    test()
