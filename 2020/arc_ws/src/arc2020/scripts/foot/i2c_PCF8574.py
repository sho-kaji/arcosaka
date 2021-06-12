#!/usr/bin/env python
# -*- coding: utf-8 -*-
# datasheet : PCF8574 https://www.ti.com/document-viewer/PCF8574/datasheet/specifications#SCPS0689014
"""

"""

import os
import time
#if os.name == 'posix':
import pigpio
__pigio__ = 0

I2CBUSS0 = 0 # use i2c bus 0 ( GPIO 0 and 1)
PCF8574_ADDR = 0x40 #nead confirming by i2cdetect : i2cdetect -y 0
READ_ADDR = [0x41,0x43,0x45,0x47,0x49,0x4B,0x4D,0x4F]



class I2CPCF8574(object):
    """
    Lineセンサークラス
    """
    def __init__(self, is_debug=False):
        
        #if self.is_notdebug:
        if __pigio__:
            # initialize gpio
            self.pic = pigpio.pi()
            
        print "GO"

    def read(self):
        result = []
        for i in range(len(READ_ADDR)):
            result.append(1)
        if __pigio__:
            self.i2c_handle = self.pic.i2c_open(I2CBUSS0,PCF8574_ADDR)
            for i in range(len(READ_ADDR)):
                word = self.pic.i2c_read_byte_data(self.i2c_handle, READ_ADDR[i])
                result[i] = word
            self.i2c_handle = self.pic.i2c_close(I2CBUSS0,PCF8574_ADDR)
        return result

    #end __init__

if __name__ == '__main__':
    pcf8574 = I2CPCF8574(False,(13,5))

    while 1:
        for i in range(len(READ_ADDR)):
            print (i + ":" + pcf8574.read()[i])
        time.sleep(0.1)
