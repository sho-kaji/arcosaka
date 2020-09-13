#!/usr/bin/python
# -*- coding: utf-8 -*-
# pylint: disable=E1101,C0325
"""
モーターテスト
"""

import mortor
import abh

from params import TARGET
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
# end main_dc


def main_dccut():
    """
    刃DCモーターテスト
    """
    mcc = mortor.DcMortorClass()
    while True:
        try:
            val = input('number(0 ~ 100):')
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
# end main_dccut


def main_servo():
    """
    サーボモーターテスト
    """
    print("15号(%d) 刈りん(%d)" % (TARGET.TOMATO, TARGET.GRASS))
    tgt = input('target:')
    int_tgt = TARGET(tgt)

    if (int_tgt == TARGET.SIDE_SPROUT) or (int_tgt == TARGET.TOMATO) or (int_tgt == TARGET.GRASS):
        smc = mortor.ServoMortorClass()
    else:
        return
    if (int_tgt == TARGET.SIDE_SPROUT) or (int_tgt == TARGET.TOMATO):
        for channel in enumerate(CHANNEL_M):
            pass
            # smc.posinit(channel)
    elif int_tgt == TARGET.GRASS:
        for channel in enumerate(CHANNEL_K):
            pass
            # smc.posinit(channel)
    else:
        pass

    while True:
        try:
            if (int_tgt == TARGET.SIDE_SPROUT) or (int_tgt == TARGET.TOMATO):
                print("[15号]\n(0)ハンド\t(1)手首  \n(2)枝掴み\t(3)枝ねじり\n(4)添え手右\t(5)添え手左")
            elif int_tgt == TARGET.GRASS:
                print(
                    "[刈りん]\n(0)ハンド\t(1)手首  \n(2)引抜  \t(3)肘    \n(4)肩    \t(5)土台\n(6)蓋")
            else:
                break

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
# end main_servo


def main_step():
    """
    ステッピングモーターテスト
    """
    smc_handh = mortor.StepMortorClass(
        False, (PORT_HANDH_A, PORT_HANDH_B), (-LIM_HANDH_MAX, LIM_HANDH_MAX))
    smc_handv = mortor.StepMortorClass(
        False, (PORT_HANDV_A, PORT_HANDV_B), (-LIM_HANDV_MAX, LIM_HANDV_MAX))
    smc_twisth = mortor.StepMortorClass(
        False, (PORT_TWISTH_A, PORT_TWISTH_B), (-LIM_TWISTH_MAX, LIM_TWISTH_MAX))
    smc_twistv = mortor.StepMortorClass(
        False, (PORT_TWISTV_A, PORT_TWISTV_B), (-LIM_TWISTV_MAX, LIM_TWISTV_MAX))

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

            elif mortornum == 5:
                smc_handh.posinit(0)
                smc_handv.posinit(0)
                smc_twisth.posinit(0)
                smc_twistv.posinit(0)

            else:
                break

        except KeyboardInterrupt:
            print("Ctrl+Cで停止しました")
            break
        except TypeError as ex:
            print(ex)

    smc_handh.endfnc()
    smc_handv.endfnc()
    smc_twisth.endfnc()
    smc_twistv.endfnc()
# end main_step


def main_test():
    """
    モーターテスト
    """
    print("芽かき(%d) 収穫(%d) 刈りん(%d)" %
          (TARGET.SIDE_SPROUT, TARGET.TOMATO, TARGET.GRASS))
    tgt = input('target:')
    int_tgt = TARGET(tgt)

    is_loop = True

    if (int_tgt == TARGET.SIDE_SPROUT) or (int_tgt == TARGET.TOMATO) or (int_tgt == TARGET.GRASS):
        abhc = abh.AbhClass(True)
        abhc.target_now = int_tgt
        abhc.posinit()
    else:
        is_loop = False

    while is_loop:
        try:
            if int_tgt == TARGET.SIDE_SPROUT:
                print(
                    "[芽かき]\n(0)\tねじ切り垂直\t(1)\tねじ切り水平\n(2)\t添え手右・添え手左\n(3)\t枝掴み\t(4)\t枝ねじり")
                int_tmp = int(input('mode:'))
                if int_tmp == 0:
                    int_val = int(input('(0)[mm]:'))
                    abhc.move_twistv(int_val)
                elif int_tmp == 1:
                    int_val = int(input('(1)[mm]:'))
                    abhc.move_twisth(int_val)
                elif int_tmp == 2:
                    int_val = int(input('(2)[%]:'))
                    abhc.move_attach_r(int_val)
                    int_val = 100 - int_val
                    abhc.move_attach_l(int_val)
                elif int_tmp == 3:
                    int_val = int(input('(3)0/1:'))
                    abhc.move_grab(int_val * 100)
                elif int_tmp == 4:
                    int_val = int(input('(4)[%]:'))
                    abhc.move_twist(int_val)
                else:
                    break

            elif int_tgt == TARGET.TOMATO:
                print("[収穫]\n(0)\tハンド垂直\t(1)\tハンド水平\n(2)\tハンド\t(3)\t手首")
                int_tmp = int(input('mode:'))
                if int_tmp == 0:
                    pass
                elif int_tmp == 1:
                    pass
                elif int_tmp == 2:
                    pass
                elif int_tmp == 3:
                    pass
                else:
                    break
            elif int_tgt == TARGET.GRASS:
                print("[草刈り]\n(0)\t土台\t(1)\t肩\n(2)\tハンド\t(3)\t引抜")
                print("(4)\t手首\t(5)\t散布ファン\n(6)\t蓋\t(7)\t肘\n(8)\t刃")
                int_tmp = int(input('mode:'))
                if int_tmp == 0:
                    pass
                elif int_tmp == 1:
                    pass
                elif int_tmp == 2:
                    pass
                elif int_tmp == 3:
                    pass
                elif int_tmp == 4:
                    pass
                elif int_tmp == 5:
                    pass
                elif int_tmp == 6:
                    pass
                elif int_tmp == 7:
                    pass
                elif int_tmp == 8:
                    pass
                else:
                    break
            else:
                break

        except KeyboardInterrupt:
            print("Ctrl+Cで停止しました")
            break
        except TypeError as ex:
            print(ex)

# end main_test


if __name__ == '__main__':
    main_step()
