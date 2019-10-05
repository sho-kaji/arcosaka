#!/usr/bin/env python
# -*- coding: utf-8 -*-
# pylint: disable=E1101,C0325
"""
電流電圧測定IC
"""

from subprocess import call
import time
import smbus


I2C_INA226 = 0x40 #I2Cアドレス(固定)
ADDR_V = 0x02
ADDR_I = 0x04
ADDR_R = 0x05

BATT_LOW = 6.2
BATT_ERR = 10

class Ina226(object):
    """
    電流電圧測定IC値取得クラス
    """
    def __init__(self):
        self.i2c = smbus.SMBus(1)
        self.i2c.write_word_data(I2C_INA226, ADDR_R, 0x14)

        self.is_battlow = False
        self.cnt_battlow = 0
        self.v_ave = 0
        self.i_ave = 0
        self.read_v()
        self.read_i()

    def read_v(self):
        """
        電圧読み取り
        """
        word = self.i2c.read_word_data(I2C_INA226, ADDR_V) & 0xFFFF
        result = ((word << 8) & 0xFF00) + (word >> 8)
        volt = result * 1.25 / 1000
        return volt

    def read_i(self):
        """
        電流読み取り
        """
        word = self.i2c.read_word_data(I2C_INA226, ADDR_I) & 0xFFFF
        curr = ((word << 8) & 0xFF00) + (word >> 8)
        return curr

    def read_vi_loop(self):
        """
        電流電圧読み取りループ
        """
        while True:
            v_now = self.read_v()
            i_now = self.read_i() / 1000.0
            self.v_ave = (self.v_ave + v_now) / 2.0
            self.i_ave = (self.i_ave + i_now) / 2.0
            print "NOW={:.2f}[V]\t".format(v_now) + "AVE={:.2f}[V]\t".format(self.v_ave) + \
                "NOW={:.4f}[A]\t".format(i_now) + "NOW={:.4f}[A]\t".format(self.i_ave)
            if self.v_ave < BATT_LOW:
                self.cnt_battlow += 1
            else:
                self.cnt_battlow = 0

            if self.cnt_battlow > BATT_ERR:
                print "シャットダウンします"
                call('sudo shutdown -h 1', shell=True)
            else:
                pass

            time.sleep(1)


if __name__ == '__main__':
    inac = Ina226()
    inac.read_vi_loop()
