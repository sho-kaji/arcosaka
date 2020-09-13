#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
brain.py 定数ファイル
"""

from params import MODE, TARGET

# defined const

DEBUG_BRAIN = True # Brainデバッグモードフラグ

CYCLES = 60 #処理周波数

DEFAULT_MOV = 300   # 移動量default[mm]
DEFAULT_ROT = 90    # 回転量default[deg]

WAIT = 2    # 駆動終了後の待機時間[sec]

CENTER_THRESH = 10   #対象物検出時のロボット位置調整閾値[mm]
ROTATE_DIST = 400   #ポールからどのくらい手前で旋回させるかの距離[mm]

JUUGO_GOU = "15gou"     #ロボット名
KARIN_SAMA = "karin"    #ロボット名

I_AM = JUUGO_GOU  #自分は何のロボットか

# (.+)\t(.+)\t(.+) $1 = $2 # $3
