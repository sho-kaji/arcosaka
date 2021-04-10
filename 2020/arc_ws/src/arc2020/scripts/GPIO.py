#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
GPIO.py GPIO定数ファイル
"""

from enum import IntEnum

class GPIOPIN(IntEnum):
    PWM_VPOWER1     = 2
    PWM_VPOWER2     = 3
    ROT_ENCODE1     = 4
    ROT_ENCODE2     = 14
    RET_ORGSW       = 15
    RESERVED1       = 17
    STEP_MOTOR_ENBL = 18
    RESERVED2       = 27
    RESERVED3       = 22
    RESERVED4       = 23
    RESERVED5       = 24
    RESERVED6       = 10
    RESERVED7       = 9
    RESERVED8       = 25
    STEP_MOTOR_A1   = 11
    STEP_MOTOR_B1   = 8
    SHUT_DOWN_DONE  = 7
    GYRO_LINE1      = 0
    GYRO_LINE2      = 1
    SONAR_PULS      = 5
    SONAR_TRIG1     = 6
    SONAR_TRIG2     = 12
    EMERGE_STOP     = 13
    DC_MOTOR_B1     = 19
    DC_MOTOR_A1     = 16
    DC_MOTOR_B2     = 26
    DC_MOTOR_A2     = 20
    SHUT_DOWNSW     = 21

