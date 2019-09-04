#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
hand.py 定数ファイル
"""
# defined const

DEBUG_HAND = False # ハンドデバッグモードフラグ

CATCH_GRAB = 100 # 枝掴みモーター_掴む
CATCH_HAND = 100 # ハンドモーター_掴む
OFF_PLUCK = 0 # 引抜モーター_戻す
OFF_TWIST = 0 # 枝ねじりモーター_戻す
ON_PLUCK = 100 # 引抜モーター_引き抜く
ON_TWIST = 100 # 枝ねじりモーター_ねじる
RELEASE_GRAB = 0 # 枝掴みモーター_離す
RELEASE_HAND = 0 # ハンドモーター_離す

LIM_ATTACH_LL = 0 # 添え手左モーター_左制限値
LIM_ATTACH_LR = 100 # 添え手左モーター_右制限値
LIM_ATTACH_RL = 0 # 添え手右モーター_左制限値
LIM_ATTACH_RR = 100 # 添え手右モーター_右制限値

PORT_ATTACH_LR = 11 # 添え手左モーターポート番号
PORT_ATTACH_RR = 10 # 添え手右モーターポート番号
PORT_GRAB = 8 # 枝掴みモーターポート番号
PORT_HAND = 6 # ハンドモーターポート番号
PORT_PLUCK = 8 # 引抜モーターポート番号
PORT_TWIST = 9 # 枝ねじりモーターポート番号

# (.+)\t(.+)\t(.+) $1 = $2 # $3
