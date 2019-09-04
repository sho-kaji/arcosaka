#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
param.py モード定数ファイル
"""

from enum import IntEnum

class Mode(IntEnum):
    """
    動作モード
    """
    UNKNOWN = -1
    AUTO = 0
    MANUAL = 1
    DEBUG = 2

class TARGET(IntEnum):
    """
    ターゲット
    """
    UNKNOWN = -1
    GRASS = 0
    TOMATO = 1
    SIDE_SPROUT = 2
