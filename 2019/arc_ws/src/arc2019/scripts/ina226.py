#!/usr/bin/python
# -*- coding: utf-8 -*-
# pylint: disable=E1101
"""
電圧検知IC
"""

import time
import smbus

class Ina226(object):
    def __init__(self):
        self.i2c = smbus.SMBus(1)

    def read_v(self):
        """
        電圧読み取り
        """
        word = self.i2c.read_word_data(0x40, 0x02) & 0xFFFF
        result = ( (word << 8) & 0xFF00 ) + (word >> 8)
        volt = result * 1.25 / 1000
        return volt

    def read_v_loop(self):
        """
        電圧読み取りループ
        """
        while True:
            print self.read_v()
            time.sleep(1)