#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
arm.py 定数ファイル
"""
# defined const

HIGH = 1
LOW = 0


HAND_DEBUG = False #ハンドデバッグモードフラグ
HAND_CATCH = 100 #ハンドモーター_掴む
HAND_RELEASE = 0 #ハンドモーター_離す
HAND_PORT = -1 #ハンドモーターポート番号
PLUCK_ON = 100 #引抜モーター_引き抜く
PLUCK_OFF = 0 #引抜モーター_戻す
PLUCK_PORT = -1 #引抜モーターポート番号

ARM_DEBUG = False #アームデバッグモードフラグ
WRIST_F_LIM = 0 #手首モーター_前制限値
WRIST_B_LIM = 100 #手首モーター_後制限値
WRIST_B_PORT = -1 #手首モーターポート番号
ELBOW_F_LIM = 0 #肘モーター_前制限値
ELBOW_B_LIM = 100 #肘モーター_後制限値
ELBOW_B_PORT = -1 #肘モーターポート番号
SHOULD_F_LIM = 0 #肩モーター_前制限値
SHOULD_B_LIM = 100 #肩モーター_後制限値
SHOULD_B_PORT = -1 #肩モーターポート番号
BASE_L_LIM = 0 #土台モーター_左制限値
BASE_R_LIM = 100 #土台モーター_右制限値
BASE_R_PORT = -1 #土台モーターポート番号

BODY_DEBUG = False #ボディデバッグモードフラグ
BLADE_PORT = -1 #シュレッダー刃ポート番号
SPRAY_PORT = -1 #散布ファンポート番号
LID_PORT = -1 #蓋モーターポート番号
