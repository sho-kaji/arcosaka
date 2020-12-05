#!/usr/bin/python
# -*- coding: utf-8 -*-
"""
mortor.py 定数ファイル
"""

from enum import IntEnum

# defined const


ADDR_PWM = 0x41

# for move_servo
SERVO_FREQ = 60
SERVO_MIN = 100
SERVO_MAX = 650

SRV0MAX_K = 650 # 草刈りサーボ0最大値
SRV0MIN_K = 170 # 草刈りサーボ0最小値

# 書き換え不可ここから
SERVO_MIN_K = (
    SRV0MIN_K
)

SERVO_MAX_K = (
    SRV0MAX_K
)
# 書き換え不可ここまで


# for move_step
class STEPROTATE(IntEnum):
    """
    ステップモーター回転方向
    """
    MINUS = -1
    STOP = 0
    PLUS = 1

STEP_1PULSE = 81 / 1000.0  # 1パルス距離[mm]
STEP_FREQ = 160  # ステッピングモーター周波数[Hz]
STEP_DUTY = 50  # ステッピングモーターDuty[%]
STEP_MAX = 3500
STEP_MIN = 0


# (.+)\t(.+)\t(.+) $1 = $2 # $3
