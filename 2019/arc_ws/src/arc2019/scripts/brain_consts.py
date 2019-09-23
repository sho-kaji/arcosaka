#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
brain.py 定数ファイル
"""

from params import Mode, TARGET

# defined const

DEBUG_BRAIN = False # Brainデバッグモードフラグ

CYCLES = 60 #処理周波数

DEFAULT_MOV = 300   # 移動量default[mm]
DEFAULT_ROT = 90    # 回転量default[deg]

WAIT = 2    # 駆動終了後の待機時間[sec]

CENTER_THRESH = 10   #[mm]

# (.+)\t(.+)\t(.+) $1 = $2 # $3
