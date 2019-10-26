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


class STEPROTATE(IntEnum):
    """
    ステップモーター回転方向
    """
    MINUS = -1
    STOP = 0
    PLUS = 1


ADDR_PWM = 0x41

# for move_dc
DC_FREQ = 20 * 1000  # DCモーター周波数[Hz]
DC_DUTY = 40  # DCモーターDuty[%]
DC_PLUS = 2  # DCモーターDuty1周期変化量[%]

# for move_servo
SERVO_FREQ = 60
SERVO_MIN = 100
SERVO_MAX = 650

SRV0MAX_M = 400 # 芽かきサーボ0最大値
SRV0SRT_M = 400 # 芽かきサーボ0初期値
SRV0MIN_M = 110 # 芽かきサーボ0最小値
SRV1MAX_M = 500 # 芽かきサーボ1最大値
SRV1SRT_M = 450 # 芽かきサーボ1初期値
SRV1MIN_M = 350 # 芽かきサーボ1最小値
SRV2MAX_M = 420 # 芽かきサーボ2最大値
SRV2SRT_M = 420 # 芽かきサーボ2初期値
SRV2MIN_M = 400 # 芽かきサーボ2最小値
SRV3MAX_M = 450 # 芽かきサーボ3最大値
SRV3SRT_M = 450 # 芽かきサーボ3初期値
SRV3MIN_M = 150 # 芽かきサーボ3最小値
SRV4MAX_M = 510 # 芽かきサーボ4最大値
SRV4SRT_M = 423 # 芽かきサーボ4初期値
SRV4MIN_M = 150 # 芽かきサーボ4最小値
SRV5MAX_M = 640 # 芽かきサーボ5最大値
SRV5SRT_M = 390 # 芽かきサーボ5初期値
SRV5MIN_M = 310 # 芽かきサーボ5最小値

SRV0MAX_K = 500 # 草刈りサーボ0最大値
SRV0SRT_K = 500 # 草刈りサーボ0初期値
SRV0MIN_K = 170 # 草刈りサーボ0最小値
SRV1MAX_K = 460 # 草刈りサーボ1最大値
SRV1SRT_K = 460 # 草刈りサーボ1初期値
SRV1MIN_K = 200 # 草刈りサーボ1最小値
SRV2MAX_K = 700 # 草刈りサーボ2最大値
SRV2SRT_K = 700 # 草刈りサーボ2初期値
SRV2MIN_K = 90 # 草刈りサーボ2最小値
SRV3MAX_K = 530 # 草刈りサーボ3最大値
SRV3SRT_K = 530 # 草刈りサーボ3初期値
SRV3MIN_K = 200 # 草刈りサーボ3最小値
SRV4MAX_K = 500 # 草刈りサーボ4最大値
SRV4SRT_K = 500 # 草刈りサーボ4初期値
SRV4MIN_K = 210 # 草刈りサーボ4最小値
SRV5MAX_K = 480 # 草刈りサーボ5最大値
SRV5SRT_K = 480 # 草刈りサーボ5初期値
SRV5MIN_K = 305 # 草刈りサーボ5最小値
SRV6MAX_K = 700 # 草刈りサーボ6最大値
SRV6SRT_K = 700 # 草刈りサーボ6初期値
SRV6MIN_K = 90 # 草刈りサーボ6最小値
SRV7MAX_K = 700 # 草刈りサーボ7最大値
SRV7SRT_K = 700 # 草刈りサーボ7初期値
SRV7MIN_K = 90 # 草刈りサーボ7最小値

# 書き換え不可ここから
SERVO_MIN_M = (
    SRV0MIN_M,
    SRV1MIN_M,
    SRV2MIN_M,
    SRV3MIN_M,
    SRV4MIN_M,
    SRV5MIN_M
)
SERVO_SRT_M = (
    SRV0SRT_M,
    SRV1SRT_M,
    SRV2SRT_M,
    SRV3SRT_M,
    SRV4SRT_M,
    SRV5SRT_M
)
SERVO_MAX_M = (
    SRV0MAX_M,
    SRV1MAX_M,
    SRV2MAX_M,
    SRV3MAX_M,
    SRV4MAX_M,
    SRV5MAX_M
)
SERVO_MIN_K = (
    SRV0MIN_K,
    SRV1MIN_K,
    SRV2MIN_K,
    SRV3MIN_K,
    SRV4MIN_K,
    SRV5MIN_K,
    SRV6MIN_K,
    SRV7MIN_K
)
SERVO_SRT_K = (
    SRV0SRT_K,
    SRV1SRT_K,
    SRV2SRT_K,
    SRV3SRT_K,
    SRV4SRT_K,
    SRV5SRT_K,
    SRV6SRT_K,
    SRV7SRT_K
)
SERVO_MAX_K = (
    SRV0MAX_K,
    SRV1MAX_K,
    SRV2MAX_K,
    SRV3MAX_K,
    SRV4MAX_K,
    SRV5MAX_K,
    SRV6MAX_K,
    SRV7MAX_K
)
# 書き換え不可ここまで

# for move_step
STEP_1PULSE = 81 / 1000.0  # 1パルス距離[mm]
STEP_FREQ = 160  # ステッピングモーター周波数[Hz]
STEP_DUTY = 50  # ステッピングモーターDuty[%]

# (.+)\t(.+)\t(.+) $1 = $2 # $3
