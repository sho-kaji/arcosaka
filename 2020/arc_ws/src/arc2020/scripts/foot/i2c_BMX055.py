#!/usr/bin/env python
# -*- coding: utf-8 -*-
# datasheet : BMX055 https://www.mouser.jp/datasheet/2/783/BST_BMX055_DS000-1509552.pdf
# refer : https://qiita.com/hiro-han/items/ca881a6c76559af9f57a
#         https://akizukidenshi.com/catalog/g/gK-13010/
#         https://akizukidenshi.com/download/ds/akizuki/BMX055_20180510.zip
"""
ライントレースセンサー
"""

import os
import time
# if os.name == 'posix':
import pigpio
from PIGPIO_SWITCH import __pigpio__

I2CBUSS0 = 0  # use i2c bus 0 ( use GPIO 0 and 1)
# Table 64 I2C address
OPEN_ADDR_ACCEL = 0x19  # nead confirming by i2cdetect : i2cdetect -y 0 // (JP1,JP2,JP3 = Openの時)
OPEN_ADDR_GYRO = 0x69  # nead confirming by i2cdetect : i2cdetect -y 0  // (JP1,JP2,JP3 = Openの時)
OPEN_ADDR_MAG = 0x13  # nead confirming by i2cdetect : i2cdetect -y 0   // (JP1,JP2,JP3 = Openの時)
READ_ADDR_ACCEL = 0x02
READ_ADDR_GYRO = 0x02
READ_ADDR_MAG = 0x42
XYZ = [0, 0, 0]


class I2CBMX055(object):
    """
    9軸センサークラス
    """

    def __init__(self, is_debug=False):

        # if self.is_notdebug:
        if __pigio__:
            # initialize gpio
            self.pic = pigpio.pi()

        print ("GO")
    # end __init__

    def read(self,OPEN_ADDR,READ_ADDR):
        value = []
        for i in range(len(XYZ)*2):
            value.append(0)
        if __pigio__:
            self.i2c_handle = self.pic.i2c_open(I2CBUSS0, OPEN_ADDR)
            value = self.pic.i2c_read_i2c_block_data(self.i2c_handle, READ_ADDR,len(XYZ)*2)
            self.i2c_handle = self.pic.i2c_close(I2CBUSS0, OPEN_ADDR)
        return value
    # end read

    # Read data back from 0x02(02), 6 bytes
    # xGyro LSB, xGyro MSB, yGyro LSB, yGyro MSB, zGyro LSB, zGyro MSB
    def read_accel(self):
        result = []
        value = self.read(OPEN_ADDR_ACCEL, READ_ADDR_ACCEL)
        for i in range(len(XYZ)):
            result.append(1)
            result[i] = self.convert_accel(value[2*i], value[2*i+1])
        return result
    # end read_accel

    def convert_accel(self, msb, lsb):
        value = (msb * 16) + ((lsb & 0xF0) / 16)
        value = value if value < 2048 else value - 4096
        value = value * 0.00098
        return value
    # end convert_accel

    # Read data back from 0x02(02), 6 bytes
    # xGyro LSB, xGyro MSB, yGyro LSB, yGyro MSB, zGyro LSB, zGyro MSB
    def read_gyro(self):
        result = []
        value = self.read(OPEN_ADDR_GYRO, READ_ADDR_GYRO)
        for i in range(len(XYZ)):
            result.append(1)
            result[i] = self.convert_gyro(value[2*i], value[2*i+1])
        return result
    # end read_gyro

    def convert_gyro(self, msb, lsb):
        value = (msb * 256) + lsb
        value = value if value < 32767 else value - 65536
        return value
    # end convert_gyro

    # Read data back from 0x42(66), 6 bytes
    # X-Axis LSB, X-Axis MSB, Y-Axis LSB, Y-Axis MSB, Z-Axis LSB, Z-Axis MSB
    def read_mag(self):
        result = []
        value = self.read(OPEN_ADDR_MAG, READ_ADDR_MAG)
        for i in range(len(XYZ)):
            result.append(1)
            result[i] = self.convert_mag(value[2*i], value[2*i+1])
        return result
    # end read_mag

    def convert_mag(self, msb, lsb):
        value = (msb * 256) + lsb/8
        value = value if value < 4095 else value - 8192
        return value
    # end convert_gyro

if __name__ == '__main__':
    i2c9DSENSOR = I2CBMX055(False, (13, 5))

    while 1:
        accel = i2c9DSENSOR.read_accel()
        gyro  = i2c9DSENSOR.read_gyro()
        mag  = i2c9DSENSOR.read_mag()
        print ("accel" + "x:" + accel[0] + ",y:"+ accel[1] + ",z:"+ accel[2])
        print ("gyro" + "x:" + gyro[0] + ",y:"+ gyro[1] + ",z:"+ gyro[2])
        print ("mag" + "x:" + mag[0] + ",y:"+ mag[1] + ",z:"+ mag[2])
        time.sleep(0.1)
