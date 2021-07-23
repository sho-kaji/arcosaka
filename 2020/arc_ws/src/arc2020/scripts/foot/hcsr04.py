#!/usr/bin/env python
# -*- coding: utf-8 -*-
# pylint: disable=E1101,C0325
"""
距離センサー
"""

import os
import time
import threading
import pandas as pd
#if os.name == 'posix':
import pigpio

from PIGPIO_SWITCH import __pigpio__

LIMIT_LOOP_COUNT = 500
SONIC_COFFICIENT = 0.034/2#cm
D_LIST_MAX = 15

class HCSR04Class(object):
    """
    距離センサークラス
    """
    def __init__(self, is_debug=False, ports=(16, 20)):
        #self.is_notdebug = not((os.name != 'posix') or is_debug)
        #print(os.name)
        #print(self.is_notdebug)
        self.distance = 0.00000
        self.dlist=[]

        self.port_trig = ports[0]
        self.port_echo = ports[1]
        print "      trig={:2d}\t".format(self.port_trig)
        print "      echo={:2d}\t".format(self.port_echo)
        
        #if self.is_notdebug:
        if __pigpio__:
            # initialize gpio
            self.pic = pigpio.pi()
            self.pic.set_mode(self.port_trig, pigpio.OUTPUT)
            self.pic.set_mode(self.port_echo, pigpio.INPUT)
        print "GO"
    
    def read(self):
        ret = 0.0000
        if len(self.dlist) > D_LIST_MAX - 1 :
            s = pd.Series(self.dlist)
            #print(s)
            distance = s.rolling(10).mean()
            #print (distance)
            ret = distance[D_LIST_MAX -1]
        # print ("read ret" + str(ret))
        return ret

    def run(self):
        """
        距離を測定
        """
        sigoff = 0.00000
        sigon  = 0.00000
        loopcount = 0
        distance = 0.0000
        print "loop start"
        while 1:
            if __pigpio__:
                #self.pic.write(self.port_trig, pigpio.LOW)
                #time.sleep(0.3)

                self.pic.write(self.port_trig, pigpio.HIGH)
                time.sleep(0.00001)
                self.pic.write(self.port_trig, pigpio.LOW)
                #print "sigoff"
                sigoff = time.time()
                while self.pic.read(self.port_echo) == 0:
                    sigoff = time.time()
                    loopcount = loopcount + 1
                    if loopcount>LIMIT_LOOP_COUNT :
                        break

                #print "sigon"
                while self.pic.read(self.port_echo) == 1:
                    sigon = time.time()
            #print "    sigoff={:.6f}\t".format(sigoff)
            #print "     sigon={:.6f}\t".format(sigon)
            timepassed = sigon - sigoff
            #print "timepassed={:.6f}\t".format(timepassed)
            distance = timepassed * SONIC_COFFICIENT
            #distance = distance + 1.00
            # sonor max is 450 cm
            if distance < 450:
                if loopcount < LIMIT_LOOP_COUNT:
                    self.dlist.append(distance)
                    if len(self.dlist) > D_LIST_MAX:
                            self.dlist.pop(0)
    #end run

    def start(self):
        sonor_thread = threading.Thread(target=self.run)
        sonor_thread.setDaemon(True)
        sonor_thread.start()

if __name__ == '__main__':
    hcsrc = HCSR04Class(False,(13,5))

    while 1:
        print "       NOW={:.4f}[]\t".format(hcsrc.read())
        time.sleep(0.1)
