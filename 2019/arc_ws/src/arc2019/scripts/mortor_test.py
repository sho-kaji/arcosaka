#!/usr/bin/python
# -*- coding: utf-8 -*-
# pylint: disable=E1101,C0325
"""
モーターテスト
"""

import mortor
from mortor_consts import \
    DCROTATE

def main_dc():
    """
    DCモーターテスト
    """
    mcc = mortor.MortorClass()
    while True:
        try:
            val = input('number(-2～2):')
            int_tmp = int(val)
            if int_tmp == 0:
                mcc.move_dc(DCROTATE.STOP, DCROTATE.STOP)
            elif int_tmp == 1:
                mcc.move_dc(DCROTATE.CW, DCROTATE.CCW)
            elif int_tmp == 2:
                mcc.move_dc(DCROTATE.CW, DCROTATE.CW)
            elif int_tmp == -1:
                mcc.move_dc(DCROTATE.CCW, DCROTATE.CW)
            elif int_tmp == -2:
                mcc.move_dc(DCROTATE.CCW, DCROTATE.CCW)
            else:
                break

        except KeyboardInterrupt:
            print("Ctrl+Cで停止しました")
            break
        except TypeError as ex:
            print(ex)
    mcc.endfnc()

def main_dccut():
    """
    DCモーターテスト
    """
    mcc = mortor.MortorClass()
    while True:
        try:
            val = input('number(0 or 1):')
            int_tmp = int(val)
            
            if int_tmp >= 0:
                mcc.move_dc_duty(9, 11, int_tmp,0)
            else:
                break

        except KeyboardInterrupt:
            print("Ctrl+Cで停止しました")
            break
        except TypeError as ex:
            print(ex)
    mcc.endfnc()

def main_servo():
    """
    サーボモーターテスト
    """
    mcc = mortor.MortorClass()
    while True:
        try:
            port = input('  port:')
            int_port = int(port)
            if int_port < 0:
                break

            val = input('number:')
            int_tmp = int(val)
            if int_tmp < 0:
                break

            mcc.move_servo(int_port, int_tmp, False)
        except KeyboardInterrupt:
            print("Ctrl+Cで停止しました")
            break
        except TypeError as ex:
            print(ex)

    mcc.endfnc()

def main_step():
    """
    ステッピングモーターテスト
    """
    mcc = mortor.MortorClass()
    while True:
        try:
            val = input('  step:')
            int_tmp = int(val)
            if int_tmp == 0:
                break

            mcc.move_step_step(11, 8, int_tmp)

        except KeyboardInterrupt:
            print("Ctrl+Cで停止しました")
            break
        except TypeError as ex:
            print(ex)
    mcc.endfnc()

if __name__ == '__main__':
    main_step()
