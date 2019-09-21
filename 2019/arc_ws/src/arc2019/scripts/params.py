#!/usr/bin/python
# -*- coding: utf-8 -*-
"""
param.py モード定数ファイル
"""

from enum import IntEnum

class MODE(IntEnum):
    """
    動作モード
    """
    UNKNOWN = -1
    INIT = 0
    AUTO = 1
    MANUAL = 2
    DEBUG = 3

class TARGET(IntEnum):
    """
    ターゲット
    """
    UNKNOWN = -1
    GRASS = 0
    TOMATO = 1
    SIDE_SPROUT = 2

class CAMERA(IntEnum):
    """
    カメラ
    """
    UNKNOWN = -1
    MAIN = 0        #トマト、雑草、主枝
    SUB = 1         #脇芽
    POLL = 2        #ポール
