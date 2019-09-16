#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""
hand.py 定数ファイル
"""

from enum import Enum

# defined const

DEBUG_HAND = False # ハンドデバッグモードフラグ

CATCH_HAND = 100 # ハンドモーター_掴む
RELEASE_HAND = 0 # ハンドモーター_離す
LIM_WRIST_F = 0 # 手首モーター_前制限値
LIM_WRIST_B = 100 # 手首モーター_後制限値
ON_PLUCK = 100 # 引抜モーター_引き抜く
OFF_PLUCK = 0 # 引抜モーター_戻す
CATCH_GRAB = 100 # 枝掴みモーター_掴む
RELEASE_GRAB = 0 # 枝掴みモーター_離す
ON_TWIST = 100 # 枝ねじりモーター_ねじる
OFF_TWIST = 0 # 枝ねじりモーター_戻す
LIM_ATTACH_RL = 0 # 添え手右モーター_左制限値
LIM_ATTACH_RR = 100 # 添え手右モーター_右制限値
LIM_ATTACH_LL = 0 # 添え手左モーター_左制限値
LIM_ATTACH_LR = 100 # 添え手左モーター_右制限値

#サーボモーター用
CHANNEL_HAND = 0 # ハンドモーターポート番号
CHANNEL_PLUCK = 2 # 引抜モーターポート番号
CHANNEL_GRAB = 2 # 枝掴みモーターポート番号
CHANNEL_TWIST = 3 # 枝ねじりモーターポート番号
CHANNEL_ATTACH_RR = 4 # 添え手右モーターポート番号
CHANNEL_ATTACH_LR = 5 # 添え手左モーターポート番号
CHANNEL_WRIST = 1 # 手首モーターポート番号


class MOTORH(Enum):
    """
    ハンドモーター
    """
    HAND = 0 # ハンドモーター
    PLUCK = 1 # 引抜モーター
    GRAB = 2 # 枝掴みモーター
    TWIST = 3 # 枝ねじりモーター
    ATTACHR = 4 # 添え手右モーター
    ATTACHL = 5 # 添え手左モーター
    WRIST = 6 # 手首モーター

# (.+)\t(.+)\t(.+) $1 = $2 # $3
