#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
ina226.py 定数ファイル
"""

# defined const

I2C_INA226 = 0x40  # I2Cアドレス(固定)
ADDR_S = 0x00  # 設定ビットフラグアドレス
ADDR_V = 0x02  # 電圧[V]取得アドレス
ADDR_I = 0x04  # 電流[mA]取得アドレス
ADDR_R = 0x05  # 抵抗[Ω]設定アドレス

SET_RST = 0b0100
SET_AVG = 0b000
SET_VBUSCT = 0b100
SET_VSHCT = 0b100
SET_MODE = 0b111

BATT_R = 0.0015  # シャント抵抗値

BATT_VMAX = 8.4  # 電圧最大値
BATT_VLOW = 6.7  # 電圧低下アラーム設定値
BATT_IHI = 15.0  # 電流異常アラーム設定値
BATT_ERR = 10  # バッテリ情報エラー回数上限


# (.+)\t(.+)\t(.+) $1 = $2 # $3
