#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
client.py 定数ファイル
"""

from params import MODE, TARGET

# defined const

CYCLES = 60 #[Hz]

DEFAULT_MODE = MODE.AUTO
DEFAULT_TARGET = TARGET.TOMATO

# (.+)\t(.+)\t(.+) $1 = $2 # $3
