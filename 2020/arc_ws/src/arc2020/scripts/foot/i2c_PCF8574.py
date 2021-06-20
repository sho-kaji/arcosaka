#!/usr/bin/env python
# -*- coding: utf-8 -*-
# datasheet : PCF8574 https://www.ti.com/document-viewer/PCF8574/datasheet/specifications#SCPS0689014
"""

"""

import os
import time
#if os.name == 'posix':
import pigpio
from PIGPIO_SWITCH import __pigpio__

I2CBUSS0 = 0 # use i2c bus 0 ( GPIO 0 and 1)
PCF8574_ADDR = 0x20 #nead confirming by i2cdetect : i2cdetect -y 0
READ_ADDR = 0xFF
SENSOR_NUM = 8


class I2CPCF8574(object):
    """
    Lineセンサークラス
    """
    def __init__(self, is_debug=False):
        
        #if self.is_notdebug:
        if __pigpio__:
            # initialize gpio
            self.pic = pigpio.pi()
            
        print "GO"

    def read(self):
        result = []
        for i in range(SENSOR_NUM):
            result.append(1)
        if __pigpio__:
            self.i2c_handle = self.pic.i2c_open(I2CBUSS0,PCF8574_ADDR)
            word = self.pic.i2c_read_byte_data(self.i2c_handle, READ_ADDR)
            for i in range(SENSOR_NUM):
                result[i] = (word >> i)& 1
            self.pic.i2c_close(self.i2c_handle)
        return result

    #end __init__

if __name__ == '__main__':
    pcf8574 = I2CPCF8574()

    while 1:
        for i in range(len(READ_ADDR)):
            print (i + ":" + pcf8574.read()[i])
        time.sleep(0.1)
