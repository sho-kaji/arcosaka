#!/usr/bin/python
# -*- coding: utf-8 -*-
"""
arm.py 定数ファイル
"""

from enum import Enum,IntEnum

import pigpio

# defined const
DEBUG_HAND = False  # ハンドデバッグモードフラグ
DEBUG_ARM = False  # アームデバッグモードフラグ
DEBUG_BODY = False  # ボディデバッグモードフラグ

CHANNEL_HAND = 0  # ハンドモーターポート番号

LIM_HANDV_MAX = 4938  # ハンド垂直モーター最大値
LIM_HANDV_MIN = 0  # ハンド垂直モーター最小値

PORT_HANDV_A = 11  # ハンド垂直モーターAポート番号
PORT_HANDV_B = 8  # ハンド垂直モーターBポート番号
PORT_PWOFFSW = 21  # シャットダウンSWポート番号

CATCH_HAND = 100  # ハンドモーター_掴む
RELEASE_HAND = 0  # ハンドモーター_離す


# (.+)\t(.+)\t(.+) $1 = $2 # $3
