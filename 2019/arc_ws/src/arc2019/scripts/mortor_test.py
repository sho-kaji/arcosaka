#!/usr/bin/python
# -*- coding: utf-8 -*-
# pylint: disable=E1101,C0325
"""
モーターテスト
"""

import mortor

from abh_consts import *

from mortor_consts import \
    DCROTATE

AIN1 = 16
AIN2 = 20
BIN1 = 19
BIN2 = 26

def main_dc():
    """
    DCモーターテスト
    """
    dmc = mortor.DcMortorClass(False, (AIN1, AIN2, BIN1, BIN2))
    while True:
        try:
            val = input('number(-2～2):')
            int_tmp = int(val)
            if int_tmp == 0:
                dmc.move_dc(DCROTATE.STOP, DCROTATE.STOP)
            elif int_tmp == 1:
                dmc.move_dc(DCROTATE.CW, DCROTATE.CCW)
            elif int_tmp == 2:
                dmc.move_dc(DCROTATE.CW, DCROTATE.CW)
            elif int_tmp == -1:
                dmc.move_dc(DCROTATE.CCW, DCROTATE.CW)
            elif int_tmp == -2:
                dmc.move_dc(DCROTATE.CCW, DCROTATE.CCW)
            else:
                break

        except KeyboardInterrupt:
            print("Ctrl+Cで停止しました")
            break
        except TypeError as ex:
            print(ex)
    dmc.endfnc()

def main_dccut():
    """
    刃DCモーターテスト
    """
    mcc = mortor.DcMortorClass()
    while True:
        try:
            val = input('number(0 or 1):')
            int_tmp = int(val)

            if int_tmp >= 0:
                mcc.move_dc_duty(PORT_BLADE_A, PORT_BLADE_B, int_tmp, 0)
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
    smc = mortor.ServoMortorClass()
    while True:
        try:

            print("[15号]\n(0)ハンド\t(1)手首  \n(2)枝掴み\t(3)枝ねじり\n(4)添え手右\t(5)添え手左")
            print("[刈りん]\n(0)ハンド\t(1)手首  \n(2)引抜  \t(3)肘    \n(4)肩    \t(5)土台\n(6)蓋")
            mortornum = input('mortor:')
            int_port = int(mortornum)
            if int_port < 0:
                break

            val = input(' pulse:')
            int_tmp = int(val)
            if int_tmp < 0:
                break

            smc.move_servo_pulse(int_port, int_tmp)
        except KeyboardInterrupt:
            print("Ctrl+Cで停止しました")
            break
        except TypeError as ex:
            print(ex)

    smc.endfnc()


def main_step():
    """
    ステッピングモーターテスト
    """
    smc_handh = mortor.StepMortorClass(
        False, (PORT_HANDH_A, PORT_HANDH_B), (LIM_HANDH_MIN, LIM_HANDH_MAX))
    smc_handv = mortor.StepMortorClass(
        False, (PORT_HANDV_A, PORT_HANDV_B), (LIM_HANDV_MIN, LIM_HANDV_MAX))
    smc_twisth = mortor.StepMortorClass(
        False, (PORT_TWISTH_A, PORT_TWISTH_B), (LIM_TWISTH_MIN, LIM_TWISTH_MAX))
    smc_twistv = mortor.StepMortorClass(
        False, (PORT_TWISTV_A, PORT_TWISTV_B), (LIM_TWISTV_MIN, LIM_TWISTV_MAX))

    while True:
        try:
            print("(1)ハンド水平\n(2)ハンド垂直\n(3)ねじ切り水平\n(4)ねじ切り垂直")
            mortornum = input('mortor:')
            if mortornum < 1:
                break

            val = input('  step:')
            int_tmp = int(val)
            if int_tmp == 0:
                break

            if mortornum == 1:
                smc_handh.move_step_step(int_tmp)

            elif mortornum == 2:
                smc_handv.move_step_step(int_tmp)

            elif mortornum == 3:
                smc_twisth.move_step_step(int_tmp)

            elif mortornum == 4:
                smc_twistv.move_step_step(int_tmp)

            else:
                break

        except KeyboardInterrupt:
            print("Ctrl+Cで停止しました")
            break
        except TypeError as ex:
            print(ex)


if __name__ == '__main__':
    main_servo()
