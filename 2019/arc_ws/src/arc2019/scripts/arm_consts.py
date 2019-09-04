#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
arm.py 定数ファイル
"""

from enum import IntEnum

# defined const

DEBUG_ARM = False # アームデバッグモードフラグ

LIM_BASE_L = 0 # 土台モーター_左制限値
LIM_BASE_R = 100 # 土台モーター_右制限値
LIM_ELBOW_B = 100 # 肘モーター_後制限値
LIM_ELBOW_F = 0 # 肘モーター_前制限値
LIM_SHOULD_B = 100 # 肩モーター_後制限値
LIM_SHOULD_F = 0 # 肩モーター_前制限値
LIM_WRIST_B = 100 # 手首モーター_後制限値
LIM_WRIST_F = 0 # 手首モーター_前制限値

PORT_BASE = 11 # 土台モーターポート番号
PORT_ELBOW = 9 # 肘モーターポート番号
PORT_HANDH_A = 22 # ハンド水平モーターAポート番号
PORT_HANDH_B = 23 # ハンド水平モーターBポート番号
PORT_HANDV_A = 24 # ハンド垂直モーターAポート番号
PORT_HANDV_B = 10 # ハンド垂直モーターBポート番号
PORT_SHOULD = 10 # 肩モーターポート番号
PORT_TWISTH_A = 9 # ねじ切り水平モーターAポート番号
PORT_TWISTH_B = 25 # ねじ切り水平モーターBポート番号
PORT_TWISTV_A = 11 # ねじ切り垂直モーターAポート番号
PORT_TWISTV_B = 8 # ねじ切り垂直モーターBポート番号
PORT_WRIST = 7 # 手首モーターポート番号

class MORTORA(IntEnum):
    """
    モーター
    """
    HAND = 0 # ハンドモーター
    PLUCK = 1 # 引抜モーター
    GRAB = 2 # 枝掴みモーター
    TWIST = 3 # 枝ねじりモーター
    ATTACH = 4 # 添え手右モーター
    WRIST = 5 # 手首モーター
    ELBOW = 6 # 肘モーター
    SHOULD = 7 # 肩モーター
    BASE = 8 # 土台モーター
    TWISTV_A = 9 # ねじ切り垂直モーター
    TWISTH_A = 10 # ねじ切り水平モーター
    HANDV_A = 11 # ハンド垂直モーター
    HANDH_A = 12 # ハンド水平モーター
    LID = 13 # 蓋モーター
    SPRAY = 14 # 散布ファン
    BLADE_A = 15 # シュレッダー刃


# (.+)\t(.+)\t(.+) $1 = $2 # $3
