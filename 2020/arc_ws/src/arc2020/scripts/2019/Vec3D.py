#!/usr/bin/env python
# -*- coding: utf-8 -*-

class Vec3D(object):
    def __init__(self):
        self.x = 0
        self.y = 0
        self.z = -1000  #無効値(奥行きがマイナスになることはない)
