#!/usr/bin/env python
# -*- coding: utf-8 -*-
# pylint: disable=E1101,C0325
"""
距離センサー
"""

import os
import time
if os.name == 'posix':
    import pigpio


class HCSR04Class(object):
    """
    距離センサークラス
    """
    def __init__(self, is_debug=False, ports=(16, 20)):
        self.is_notdebug = not((os.name != 'posix') or is_debug)
        print(os.name)
        print(self.is_notdebug)

        self.port_trig = ports[0]
        self.port_echo = ports[1]

        if self.is_notdebug:
            # initialize gpio
            self.pic = pigpio.pi()
            self.pic.set_mode(self.port_trig, pigpio.OUTPUT)
            self.pic.set_mode(self.port_echo, pigpio.INPUT)

    def read(self):
        """
        距離を測定
        """
        self.pic.write(self.port_trig, pigpio.LOW)
        time.sleep(0.3)

        self.pic.write(self.port_trig, pigpio.HIGH)
        time.sleep(0.00001)
        self.pic.write(self.port_trig, pigpio.LOW)

        while self.pic.read(self.port_echo) == 0:
            sigoff = time.time()

        while self.pic.read(self.port_echo) == 1:
            sigon = time.time()

        timepassed = sigon - sigoff
        distance = timepassed / 58.0
        return distance



    #end __init__


if __name__ == '__main__':
    hcsrc = HCSR04Class(False,(14,15))

    while 1:
        print "NOW={:.4f}[]\t".format(hcsrc.read())
        time.sleep(1)
