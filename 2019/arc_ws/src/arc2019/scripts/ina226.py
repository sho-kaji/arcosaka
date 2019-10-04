#!/usr/bin/env python
# -*- coding: utf-8 -*-
# pylint: disable=E1101,C0325
"""
電流電圧測定IC
"""

import time
import smbus

ADDR_INA226 = 0x40 #I2Cアドレス(固定)

class Ina226(object):
    """
    電流電圧測定IC値取得クラス
    """
    def __init__(self):
        self.i2c = smbus.SMBus(1)
        self.i2c.write_word_data(ADDR_INA226, 0x05, 0x14)

    def read_v(self):
        """
        電圧読み取り
        """
        word = self.i2c.read_word_data(ADDR_INA226, 0x02) & 0xFFFF
        result = ((word << 8) & 0xFF00) + (word >> 8)
        volt = result * 1.25 / 1000
        return volt

    def read_i(self):
        """
        電流読み取り
        """
        word = self.i2c.read_word_data(ADDR_INA226, 0x04) & 0xFFFF
        curr = ((word << 8) & 0xFF00) + (word >> 8)
        return curr

    def read_vi_loop(self):
        """
        電流電圧読み取りループ
        """
        self.read_v()
        self.read_i()
        while True:
            print str(self.read_v()) + "[V]"
            print str(self.read_i()) + "[mA]"
            time.sleep(1)


if __name__ == '__main__':
    inac = Ina226()
    inac.read_vi_loop()
