#!/usr/bin/python
# -*- coding: utf-8 -*-
"""
arm.py 定数ファイル
"""

from enum import Enum
import pigpio

# defined const

DEBUG_ARM = False # アームデバッグモードフラグ

LIM_BASE_L = 0 # 土台モーター_左制限値
LIM_BASE_R = 100 # 土台モーター_右制限値
LIM_ELBOW_B = 100 # 肘モーター_後制限値
LIM_ELBOW_F = 0 # 肘モーター_前制限値
LIM_HANDH_MAX = 3500 # ハンド水平モーター最大値
LIM_HANDH_MIN = 0 # ハンド水平モーター最小値
LIM_HANDV_MAX = 3500 # ハンド垂直モーター最大値
LIM_HANDV_MIN = 0 # ハンド垂直モーター最小値
LIM_SHOULD_B = 100 # 肩モーター_後制限値
LIM_SHOULD_F = 0 # 肩モーター_前制限値
LIM_TWISTH_MAX = 3500 # ねじ切り水平モーター最大値
LIM_TWISTH_MIN = 0 # ねじ切り水平モーター最小値
LIM_TWISTV_MAX = 3500 # ねじ切り垂直モーター最大値
LIM_TWISTV_MIN = 0 # ねじ切り垂直モーター最小値

#サーボモーター用
CHANNEL_ELBOW = 3 # 肘モーターポート番号
CHANNEL_SHOULD = 4 # 肩モーターポート番号
CHANNEL_BASE = 5 # 土台モーターポート番号

#DC・ステッピングモーター用
PORT_TWISTV_A = 11 # ねじ切り垂直モーターAポート番号
PORT_TWISTV_B = 8 # ねじ切り垂直モーターBポート番号
PORT_TWISTH_A = 9 # ねじ切り水平モーターAポート番号
PORT_TWISTH_B = 25 # ねじ切り水平モーターBポート番号
PORT_HANDV_A = 24 # ハンド垂直モーターAポート番号
PORT_HANDV_B = 10 # ハンド垂直モーターBポート番号
PORT_HANDH_A = 22 # ハンド水平モーターAポート番号
PORT_HANDH_B = 23 # ハンド水平モーターBポート番号

PORTS_ARM = {
    PORT_HANDH_A:pigpio.OUTPUT,
    PORT_HANDH_B:pigpio.OUTPUT,
    PORT_HANDV_A:pigpio.OUTPUT,
    PORT_HANDV_B:pigpio.OUTPUT,
    PORT_TWISTH_A:pigpio.OUTPUT,
    PORT_TWISTH_B:pigpio.OUTPUT,
    PORT_TWISTV_A:pigpio.OUTPUT,
    PORT_TWISTV_B:pigpio.OUTPUT
}


class MOTORA(Enum):
    """
    アームモーター
    """
    ELBOW = 0 # 肘モーター
    SHOULD = 1 # 肩モーター
    BASE = 2 # 土台モーター
    TWISTV = 3 # ねじ切り垂直モーター
    TWISTH = 4 # ねじ切り水平モーター
    HANDV = 5 # ハンド垂直モーター
    HANDH = 6 # ハンド水平モーター

# (.+)\t(.+)\t(.+) $1 = $2 # $3
