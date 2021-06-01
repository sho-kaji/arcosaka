#!/usr/bin/env python
# -*- coding: utf-8 -*-
# pylint: disable=E1101,C0325
"""
距離センサー
"""

import os
import time
#if os.name == 'posix':
import pigpio

__pigio__ = 0

class HCSR04Class(object):
    """
    距離センサークラス
    """
    def __init__(self, is_debug=False, ports=(16, 20)):
        #self.is_notdebug = not((os.name != 'posix') or is_debug)
        print(os.name)
        #print(self.is_notdebug)

        self.port_trig = ports[0]
        self.port_echo = ports[1]
        print "      trig={:2d}\t".format(self.port_trig)
        print "      echo={:2d}\t".format(self.port_echo)
        
        #if self.is_notdebug:
        if __pigio__:
            # initialize gpio
            self.pic = pigpio.pi()
            self.pic.set_mode(self.port_trig, pigpio.OUTPUT)
            self.pic.set_mode(self.port_echo, pigpio.INPUT)
        print "GO"

    def read(self):
        """
        距離を測定
        """
        sigoff = 10
        sigon  = 21
        if __pigio__:
            self.pic.write(self.port_trig, pigpio.LOW)
            time.sleep(0.3)

            self.pic.write(self.port_trig, pigpio.HIGH)
            time.sleep(0.00001)
            self.pic.write(self.port_trig, pigpio.LOW)

            print "sigoff"
            sigoff = time.time()
            while self.pic.read(self.port_echo) == 0:
                sigoff = time.time()

            print "sigon"
            while self.pic.read(self.port_echo) == 1:
                sigon = time.time()
        #print "    sigoff={:.6f}\t".format(sigoff)
        #print "     sigon={:.6f}\t".format(sigon)
        timepassed = sigon - sigoff
        #print "timepassed={:.6f}\t".format(timepassed)
        distance = timepassed * 58.0
        # sonor max is 450 cm
        if distance >= 450:
            distance = 450
        return distance

    #end __init__

if __name__ == '__main__':
    hcsrc = HCSR04Class(False,(13,5))

    while 1:
        print "       NOW={:.4f}[]\t".format(hcsrc.read())
        time.sleep(0.1)
