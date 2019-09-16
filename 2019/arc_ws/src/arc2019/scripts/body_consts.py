#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""
body.py 定数ファイル
"""

from enum import Enum
import pigpio

# defined const

DEBUG_BODY = False # ボディデバッグモードフラグ

#サーボモーター用
CHANNEL_LID = 6 # 蓋モーターポート番号

#DC・ステッピングモーター用
PORT_SPRAY = 24 # 散布ファンポート番号
PORT_BLADE_A = 9 # シュレッダー刃Aポート番号
PORT_BLADE_B = 11 # シュレッダー刃Bポート番号
BLADE_MIMUS = -1 # シュレッダー刃_負回転
BLADE_NONE = 0 # シュレッダー刃_無回転
BLADE_PLUS = 1 # シュレッダー刃_正回転
PORT_PWOFFSW = 21 # シャットダウンSWポート番号



class MOTORB(Enum):
    """
    ボディモーター
    """
    LID = 0 # 蓋モーター
    SPRAY = 1 # 散布ファン
    BLADE = 2 # シュレッダー刃

PORTS_PWM_BODY = {
    PORT_SPRAY:pigpio.OUTPUT,
    PORT_BLADE_A:pigpio.OUTPUT,
    PORT_BLADE_B:pigpio.OUTPUT,
    PORT_PWOFFSW:pigpio.Input
}

# (.+)\t(.+)\t(.+) $1 = $2 # $3
