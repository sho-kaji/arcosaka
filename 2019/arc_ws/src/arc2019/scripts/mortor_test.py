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
    mc = mortor.MortorClass()
    while True:
        try:
            val = input('number(-2～2):')
            int_tmp = int(val)
            if int_tmp == 0:
                mc.move_dc(DCROTATE.STOP,DCROTATE.STOP)
            elif int_tmp == 1:
                mc.move_dc(DCROTATE.CW,DCROTATE.STOP)
            elif int_tmp == 2:
                mc.move_dc(DCROTATE.CW,DCROTATE.CCW)
            elif int_tmp == -1:
                mc.move_dc(DCROTATE.STOP,DCROTATE.CW)
            elif int_tmp == -2:
                mc.move_dc(DCROTATE.CCW,DCROTATE.CW)
            else:
                break

        except KeyboardInterrupt:
            print("Ctrl+Cで停止しました")
            break
        except :
            pass
    mc.endfnc()

def main_servo():
    """
    サーボモーターテスト
    """
    mc = mortor.MortorClass()
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

            mc.move_servo(0, int_tmp,False)
        except KeyboardInterrupt:
            print("Ctrl+Cで停止しました")
            break
        except :
            pass
    mc.endfnc()

def main_step():
    """
    ステッピングモーターテスト
    """
    mc = mortor.MortorClass()
    while True:
        try:
            val = input('  step:')
            int_tmp = int(val)
            if int_tmp == 0:
                break

            mc.move_step_step(11,8,int_tmp,2)

        except KeyboardInterrupt:
            print("Ctrl+Cで停止しました")
            break
        except :
            pass
    mc.endfnc()

if __name__ == '__main__':
    main_step()
