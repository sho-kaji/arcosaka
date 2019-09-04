#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
body.py 定数ファイル
"""

from enum import IntEnum

# defined const

DEBUG_BODY = False # ボディデバッグモードフラグ

PORT_LID = 12 # 蓋モーターポート番号
PORT_SPRAY = 24 # 散布ファンポート番号
PORT_BLADE_A = 9 # シュレッダー刃Aポート番号
PORT_BLADE_B = 11 # シュレッダー刃Bポート番号
PORT_PWOFFSW = 21 # シャットダウンSWポート番号

class MORTORB(IntEnum):
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
