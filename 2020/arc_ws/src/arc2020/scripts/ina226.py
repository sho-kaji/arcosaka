#!/usr/bin/env python
# -*- coding: utf-8 -*-
# pylint: disable=E1101,C0325
"""
電流電圧測定IC
"""

from subprocess import call
import time
import smbus
import mortor

from ina226_consts import *


class Ina226Class(object):
    """
    電流電圧測定IC値取得クラス
    """

    def __init__(self):
        self.is_enable = False
        try:
            self.smc = mortor.ServoMortorClass(False)
            self.i2c = smbus.SMBus(1)
            # ICの設定
            setdata = (SET_RST << 12) + (SET_AVG << 9) + \
                (SET_VBUSCT << 6) + (SET_VSHCT << 3) + SET_MODE
            setdata = ((setdata << 8) & 0xFF00) + (setdata >> 8)
            self.i2c.write_word_data(I2C_INA226, ADDR_S, setdata)
            # キャリブレーションレジスタの設定
            # 0.00512/((0.0015[mΩ])*0.001)
            setdata = int(0.00512/((BATT_R)*0.001))
            setdata = ((setdata << 8) & 0xFF00) + (setdata >> 8)
            self.i2c.write_word_data(I2C_INA226, ADDR_R, setdata)
            self.is_enable = True
            # 1回目は変な値をとるときが多いので...
            self.read_v()
            self.read_i()
        except IOError:
            self.is_enable = False

        finally:
            self.is_battlow = False
            self.cnt_battlow = 0
            self.v_ave = 0
            self.i_ave = 0
            self.i_sgm = 0
    # end __init__

    def read_v(self):
        """
        電圧読み取り
        """
        volt = 0.0
        if self.is_enable:
            word = self.i2c.read_word_data(I2C_INA226, ADDR_V) & 0xFFFF
            result = ((word << 8) & 0xFF00) + (word >> 8)
            volt = result * 1.25 / 1000

        return volt
    # end read_v

    def read_i(self):
        """
        電流読み取り
        """
        curr = 0.0
        if self.is_enable:
            word = self.i2c.read_word_data(I2C_INA226, ADDR_I) & 0xFFFF
            curr = ((word << 8) & 0xFF00) + (word >> 8)
        return curr
    # end read_i

    def read_b(self):
        """
        バッテリー残量読み取り
        """
        batt = 0.0
        if self.is_enable:
            batt_v = self.read_i()
            batt = ((batt_v - BATT_VLOW) / (BATT_VMAX - BATT_VLOW)) * 100
        return batt

    def read_vi_loop(self):
        """
        電流電圧読み取りループ
        """
        v_aves = []
        i_aves = []
        while self.is_enable:
            v_now = self.read_v()
            v_aves.append(v_now)
            if len(v_aves) > 100:
                del v_aves[0]
            self.v_ave = sum(v_aves) / len(v_aves)

            i_now = self.read_i() / 1000.0
            self.i_sgm += i_now
            i_aves.append(i_now)
            if len(i_aves) > 100:
                del i_aves[0]
            self.i_ave = sum(i_aves) / len(i_aves)

            print "NOW={:.2f}[V]\t".format(v_now) + "AVE={:.2f}[V]\t".format(self.v_ave) + \
                "NOW={:.4f}[A]\t".format(i_now) + "AVE={:.4f}[A]\t".format(self.i_ave) + \
                "SGM={:.4f}[Asec]\t".format(self.i_sgm)

            if self.v_ave < BATT_VLOW:
                self.cnt_battlow += 1
                print "電圧低下(" + str(self.cnt_battlow) + ")"
            elif self.i_ave > BATT_IHI:
                # self.cnt_battlow += 1
                print "電流異常(" + str(self.cnt_battlow) + ")"
            else:
                self.cnt_battlow = 0

            if self.cnt_battlow > BATT_ERR:
                self.endfnc()

            else:
                pass

            time.sleep(1)
    # end read_vi_loop

    def endfnc(self):
        """
        終了処理
        """
        self.smc.endfnc()
        print "1分後にシャットダウンします"
        call('sudo shutdown -h 1', shell=True)
        self.is_enable = False
    # end endfnc


if __name__ == '__main__':
    inac = Ina226Class()
    inac.read_vi_loop()
