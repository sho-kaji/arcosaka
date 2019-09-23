#!/usr/bin/python
# -*- coding: utf-8 -*-
"""
mortor.py 定数ファイル
"""

from enum import IntEnum

# defined const

class DCROTATE(IntEnum):
    """
    DCモーター回転方向
    """
    CCW = -1
    STOP = 0
    CW = 1

ADDR_PWM = 0x41 #
ADDR_VOLT = 0x40 #

# for move_dc
DC_FREQ = 20 * 1000 # DCモーター周波数[Hz]
DC_DUTY = 60 # DCモーターDuty[%]
DC_PLUS = 2 # DCモーターDuty1周期変化量[%]

# for move_servo
SERVO_FREQ = 60
SERVO_MIN = 170
SERVO_MAX = 440

SERVO_MIN_40K = 240
SERVO_MAX_40K = 550

# for move_step
STEP_1PULSE = 10.0 # 1パルス距離[mm]
STEP_FREQ = 160 # ステッピングモーター周波数[Hz]
STEP_DUTY = 50 # ステッピングモーターDuty[%]

# (.+)\t(.+)\t(.+) $1 = $2 # $3
