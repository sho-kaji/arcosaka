#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
GPIO.py GPIO定数ファイル
"""

from enum import IntEnum

class GPIO(IntEnum):
    PWM_VPOWER1 = 2
    PWM_VPOWER2 = 3
    SONAR1_TRIG = 4
    SONAR2_TRIG = 14
    SONAR2_PULS = 15
    SONAR1_PULS = 17
    STEP_MOTOR1 = 18
    STEP_MOTOR2 = 27
    STEP_MOTOR3 = 22
    SONAR3_TRIG = 23
    SONAR3_PULS = 24
    RESERVED1   = 10
    RESERVED2   = 9
    STEP_MOTOR4 = 25
    ROT_ENCODE1 = 11
    ROT_ENCODE2 = 8
    RESERVED3   = 7
    GYRO_LINE1  = 0
    GYRO_LINE2  = 1
    SONAR4_TRIG = 5
    SONAR4_PULS = 6
    STEP_MOTOR5 = 12
    EMERGE_STOP = 13
    DC_MOTOR_B1 = 19
    DC_MOTOR_A1 = 16
    DC_MOTOR_B2 = 26
    DC_MOTOR_A2 = 20
    SHUT_DOWNSW = 21
    
