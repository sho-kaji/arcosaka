#!/usr/bin/python
# -*- coding: utf-8 -*-
"""
abh.py 定数ファイル
"""

from enum import Enum,IntEnum

import pigpio

# defined const
DEBUG_HAND = False  # ハンドデバッグモードフラグ
DEBUG_ARM = False  # アームデバッグモードフラグ
DEBUG_BODY = False  # ボディデバッグモードフラグ

LIM_WRIST_F = 0  # 手首モーター_前制限値
LIM_WRIST_B = 100  # 手首モーター_後制限値
LIM_ATTACH_RL = 0  # 添え手右モーター_左制限値
LIM_ATTACH_RR = 100  # 添え手右モーター_右制限値
LIM_ATTACH_LL = 0  # 添え手左モーター_左制限値
LIM_ATTACH_LR = 100  # 添え手左モーター_右制限値
LIM_BASE_L = 0  # 土台モーター_左制限値
LIM_BASE_R = 100  # 土台モーター_右制限値
LIM_ELBOW_B = 100  # 肘モーター_後制限値
LIM_ELBOW_F = 0  # 肘モーター_前制限値
LIM_HANDH_MAX = 3500  # ハンド水平モーター最大値
LIM_HANDH_MIN = 0  # ハンド水平モーター最小値
LIM_HANDV_MAX = 3500  # ハンド垂直モーター最大値
LIM_HANDV_MIN = 0  # ハンド垂直モーター最小値
LIM_SHOULD_B = 100  # 肩モーター_後制限値
LIM_SHOULD_F = 0  # 肩モーター_前制限値
LIM_TWISTH_MAX = 3500  # ねじ切り水平モーター最大値
LIM_TWISTH_MIN = 0  # ねじ切り水平モーター最小値
LIM_TWISTV_MAX = 3500  # ねじ切り垂直モーター最大値
LIM_TWISTV_MIN = 0  # ねじ切り垂直モーター最小値
LIM_LID_MAX = 100  # 蓋モーター最大値
LIM_LID_MIN = 0  # 蓋モーター最小値


CHANNEL_HAND = 0  # ハンドモーターポート番号
CHANNEL_WRIST = 1  # 手首モーターポート番号
CHANNEL_PLUCK = 2  # 引抜モーターポート番号
CHANNEL_GRAB = 2  # 枝掴みモーターポート番号
CHANNEL_TWIST = 3  # 枝ねじりモーターポート番号
CHANNEL_ATTACH_RR = 4  # 添え手右モーターポート番号
CHANNEL_ATTACH_LR = 5  # 添え手左モーターポート番号
CHANNEL_BASE = 5  # 土台モーターポート番号
CHANNEL_ELBOW = 3  # 肘モーターポート番号
CHANNEL_SHOULD = 4  # 肩モーターポート番号
CHANNEL_LID = 6  # 蓋モーターポート番号

PORT_HANDH_A = 9  # ハンド水平モーターAポート番号
PORT_HANDH_B = 25  # ハンド水平モーターBポート番号
PORT_HANDV_A = 11  # ハンド垂直モーターAポート番号
PORT_HANDV_B = 8  # ハンド垂直モーターBポート番号
PORT_TWISTH_A = 24  # ねじ切り水平モーターAポート番号
PORT_TWISTH_B = 10  # ねじ切り水平モーターBポート番号
PORT_TWISTV_A = 22  # ねじ切り垂直モーターAポート番号
PORT_TWISTV_B = 23  # ねじ切り垂直モーターBポート番号
PORT_SPRAY = 24  # 散布ファンポート番号
PORT_BLADE_A = 9  # シュレッダー刃Aポート番号
PORT_BLADE_B = 11  # シュレッダー刃Bポート番号
PORT_PWOFFSW = 21  # シャットダウンSWポート番号

CATCH_HAND = 100  # ハンドモーター_掴む
RELEASE_HAND = 0  # ハンドモーター_離す
ON_PLUCK = 100  # 引抜モーター_引き抜く
OFF_PLUCK = 0  # 引抜モーター_戻す
CATCH_GRAB = 100  # 枝掴みモーター_掴む
RELEASE_GRAB = 0  # 枝掴みモーター_離す
ON_TWIST = 100  # 枝ねじりモーター_ねじる
OFF_TWIST = 0  # 枝ねじりモーター_戻す
ON_SPRAY = 40  # 散布ファン最大値
OFF_SPRAY = 0  # 散布ファン最小値

BLADE_MIMUS = -1  # シュレッダー刃_負回転
BLADE_NONE = 0  # シュレッダー刃_無回転
BLADE_PLUS = 1  # シュレッダー刃_正回転

class MOTORA(Enum):
    """
    アームモーター
    """
    ELBOW = 0  # 肘モーター
    SHOULD = 1  # 肩モーター
    BASE = 2  # 土台モーター
    TWISTV = 3  # ねじ切り垂直モーター
    TWISTH = 4  # ねじ切り水平モーター
    HANDV = 5  # ハンド垂直モーター
    HANDH = 6  # ハンド水平モーター


class MOTORB(Enum):
    """
    ボディモーター
    """
    LID = 0  # 蓋モーター
    SPRAY = 1  # 散布ファン
    BLADE = 2  # シュレッダー刃


class MOTORH(Enum):
    """
    ハンドモーター
    """
    HAND = 0  # ハンドモーター
    PLUCK = 1  # 引抜モーター
    GRAB = 2  # 枝掴みモーター
    TWIST = 3  # 枝ねじりモーター
    ATTACHR = 4  # 添え手右モーター
    ATTACHL = 5  # 添え手左モーター
    WRIST = 6  # 手首モーター
    
class CHANNEL_K(IntEnum):
    """
    草刈りサーボ
    """
    HAND = CHANNEL_HAND = 0  # ハンドモーターポート番号
    WRIST = CHANNEL_WRIST = 1  # 手首モーターポート番号
    PLUCK = CHANNEL_PLUCK = 2  # 引抜モーターポート番号
    ELBOW = CHANNEL_ELBOW = 3  # 肘モーターポート番号
    SHOULD = CHANNEL_SHOULD = 4  # 肩モーターポート番号
    BASE = CHANNEL_BASE = 5  # 土台モーターポート番号
    LID = CHANNEL_LID = 6  # 蓋モーターポート番号

class CHANNEL_M(IntEnum):
    """
    芽かきサーボ
    """
    HAND = CHANNEL_HAND = 0  # ハンドモーターポート番号
    WRIST = CHANNEL_WRIST = 1  # 手首モーターポート番号
    GRAB = CHANNEL_GRAB = 2  # 枝掴みモーターポート番号
    TWIST = CHANNEL_TWIST = 3  # 枝ねじりモーターポート番号
    ATTACH_R = CHANNEL_ATTACH_RR = 4  # 添え手右モーターポート番号
    CHANNEL_ATTACH_L = CHANNEL_ATTACH_LR = 5  # 添え手左モーターポート番号


# (.+)\t(.+)\t(.+) $1 = $2 # $3
