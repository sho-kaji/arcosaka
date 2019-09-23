#!/usr/bin/env python
# -*- coding: utf-8 -*-

# 標準ライブラリ
import math

# 3次元座標の変換用クラス
# 高速化のためfor文を使わずベタ書き

# 平行移動
# |   1     0     0    Tx   |    | x |
# |   0     1     0    Ty   | \/ | y |
# |   0     0     1    Tz   | /\ | z |
# |   0     0     0     1   |    | 1 |
#
# X軸回転
# |   1     0     0     0   |    | x |
# |   0    cosθ -sinθ   0   | \/ | y |
# |   0    sinθ  cosθ   0   | /\ | z |
# |   0     0     0     1   |    | 1 |
#
# Y軸回転
# |  cosθ   0    sinθ   0   |    | x |
# |   0     1     0     0   | \/ | y |
# | -sinθ   0    cosθ   0   | /\ | z |
# |   0     0     0     1   |    | 1 |
#
# Z軸回転
# |  cosθ -sinθ   0     0   |    | x |
# |  sinθ  cosθ   0     0   | \/ | y |
# |   0     0     1     0   | /\ | z |
# |   0     0     0     1   |    | 1 |
class Transform3D(object):
    def __init__(self):
        # 4x4の単位行列を作成
        self.mat = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]

    # 行列の値をセットする valuesは4x4の2次元配列
    def set(values):
        self.mat[0][0] = values[0][0]
        self.mat[0][1] = values[0][1]
        self.mat[0][2] = values[0][2]
        self.mat[0][3] = values[0][3]
        self.mat[1][0] = values[1][0]
        self.mat[1][1] = values[1][1]
        self.mat[1][2] = values[1][2]
        self.mat[1][3] = values[1][3]
        self.mat[2][0] = values[2][0]
        self.mat[2][1] = values[2][1]
        self.mat[2][2] = values[2][2]
        self.mat[2][3] = values[2][3]
        self.mat[3][0] = values[3][0]
        self.mat[3][1] = values[3][1]
        self.mat[3][2] = values[3][2]
        self.mat[3][3] = values[3][3]

    # 積算　自分x相手
    def mulR(self, other):
        # 結果格納用
        res = [[0 for m in range(4)] for n in range(4)]

        # 1行目
        res[0][0] = self.mat[0][0]*other.mat[0][0] + self.mat[0][1]*other.mat[1][0] + self.mat[0][2]*other.mat[2][0] + self.mat[0][3]*other.mat[3][0]
        res[0][1] = self.mat[0][0]*other.mat[0][1] + self.mat[0][1]*other.mat[1][1] + self.mat[0][2]*other.mat[2][1] + self.mat[0][3]*other.mat[3][1]
        res[0][2] = self.mat[0][0]*other.mat[0][2] + self.mat[0][1]*other.mat[1][2] + self.mat[0][2]*other.mat[2][2] + self.mat[0][3]*other.mat[3][2]
        res[0][3] = self.mat[0][0]*other.mat[0][3] + self.mat[0][1]*other.mat[1][3] + self.mat[0][2]*other.mat[2][3] + self.mat[0][3]*other.mat[3][3]

        # 2行目
        res[1][0] = self.mat[1][0]*other.mat[0][0] + self.mat[1][1]*other.mat[1][0] + self.mat[1][2]*other.mat[2][0] + self.mat[1][3]*other.mat[3][0]
        res[1][1] = self.mat[1][0]*other.mat[0][1] + self.mat[1][1]*other.mat[1][1] + self.mat[1][2]*other.mat[2][1] + self.mat[1][3]*other.mat[3][1]
        res[1][2] = self.mat[1][0]*other.mat[0][2] + self.mat[1][1]*other.mat[1][2] + self.mat[1][2]*other.mat[2][2] + self.mat[1][3]*other.mat[3][2]
        res[1][3] = self.mat[1][0]*other.mat[0][3] + self.mat[1][1]*other.mat[1][3] + self.mat[1][2]*other.mat[2][3] + self.mat[1][3]*other.mat[3][3]

        # 3行目
        res[2][0] = self.mat[2][0]*other.mat[0][0] + self.mat[2][1]*other.mat[1][0] + self.mat[2][2]*other.mat[2][0] + self.mat[2][3]*other.mat[3][0]
        res[2][1] = self.mat[2][0]*other.mat[0][1] + self.mat[2][1]*other.mat[1][1] + self.mat[2][2]*other.mat[2][1] + self.mat[2][3]*other.mat[3][1]
        res[2][2] = self.mat[2][0]*other.mat[0][2] + self.mat[2][1]*other.mat[1][2] + self.mat[2][2]*other.mat[2][2] + self.mat[2][3]*other.mat[3][2]
        res[2][3] = self.mat[2][0]*other.mat[0][3] + self.mat[2][1]*other.mat[1][3] + self.mat[2][2]*other.mat[2][3] + self.mat[2][3]*other.mat[3][3]

        # 4行目
        res[3][0] = self.mat[3][0]*other.mat[0][0] + self.mat[3][1]*other.mat[1][0] + self.mat[3][2]*other.mat[2][0] + self.mat[3][3]*other.mat[3][0]
        res[3][1] = self.mat[3][0]*other.mat[0][1] + self.mat[3][1]*other.mat[1][1] + self.mat[3][2]*other.mat[2][1] + self.mat[3][3]*other.mat[3][1]
        res[3][2] = self.mat[3][0]*other.mat[0][2] + self.mat[3][1]*other.mat[1][2] + self.mat[3][2]*other.mat[2][2] + self.mat[3][3]*other.mat[3][2]
        res[3][3] = self.mat[3][0]*other.mat[0][3] + self.mat[3][1]*other.mat[1][3] + self.mat[3][2]*other.mat[2][3] + self.mat[3][3]*other.mat[3][3]

        # 計算結果反映
        self.set(res)

    # 積算　相手x自分
    def mulL(self, other):
        # 結果格納用
        res = [[0 for m in range(4)] for n in range(4)]

        # 1行目
        res[0][0] = other.mat[0][0]*self.mat[0][0] + other.mat[0][1]*self.mat[1][0] + other.mat[0][2]*self.mat[2][0] + other.mat[0][3]*self.mat[3][0]
        res[0][1] = other.mat[0][0]*self.mat[0][1] + other.mat[0][1]*self.mat[1][1] + other.mat[0][2]*self.mat[2][1] + other.mat[0][3]*self.mat[3][1]
        res[0][2] = other.mat[0][0]*self.mat[0][2] + other.mat[0][1]*self.mat[1][2] + other.mat[0][2]*self.mat[2][2] + other.mat[0][3]*self.mat[3][2]
        res[0][3] = other.mat[0][0]*self.mat[0][3] + other.mat[0][1]*self.mat[1][3] + other.mat[0][2]*self.mat[2][3] + other.mat[0][3]*self.mat[3][3]

        # 2行目
        res[1][0] = other.mat[1][0]*self.mat[0][0] + other.mat[1][1]*self.mat[1][0] + other.mat[1][2]*self.mat[2][0] + other.mat[1][3]*self.mat[3][0]
        res[1][1] = other.mat[1][0]*self.mat[0][1] + other.mat[1][1]*self.mat[1][1] + other.mat[1][2]*self.mat[2][1] + other.mat[1][3]*self.mat[3][1]
        res[1][2] = other.mat[1][0]*self.mat[0][2] + other.mat[1][1]*self.mat[1][2] + other.mat[1][2]*self.mat[2][2] + other.mat[1][3]*self.mat[3][2]
        res[1][3] = other.mat[1][0]*self.mat[0][3] + other.mat[1][1]*self.mat[1][3] + other.mat[1][2]*self.mat[2][3] + other.mat[1][3]*self.mat[3][3]

        # 3行目
        res[2][0] = other.mat[2][0]*self.mat[0][0] + other.mat[2][1]*self.mat[1][0] + other.mat[2][2]*self.mat[2][0] + other.mat[2][3]*self.mat[3][0]
        res[2][1] = other.mat[2][0]*self.mat[0][1] + other.mat[2][1]*self.mat[1][1] + other.mat[2][2]*self.mat[2][1] + other.mat[2][3]*self.mat[3][1]
        res[2][2] = other.mat[2][0]*self.mat[0][2] + other.mat[2][1]*self.mat[1][2] + other.mat[2][2]*self.mat[2][2] + other.mat[2][3]*self.mat[3][2]
        res[2][3] = other.mat[2][0]*self.mat[0][3] + other.mat[2][1]*self.mat[1][3] + other.mat[2][2]*self.mat[2][3] + other.mat[2][3]*self.mat[3][3]

        # 4行目
        res[3][0] = other.mat[3][0]*self.mat[0][0] + other.mat[3][1]*self.mat[1][0] + other.mat[3][2]*self.mat[2][0] + other.mat[3][3]*self.mat[3][0]
        res[3][1] = other.mat[3][0]*self.mat[0][1] + other.mat[3][1]*self.mat[1][1] + other.mat[3][2]*self.mat[2][1] + other.mat[3][3]*self.mat[3][1]
        res[3][2] = other.mat[3][0]*self.mat[0][2] + other.mat[3][1]*self.mat[1][2] + other.mat[3][2]*self.mat[2][2] + other.mat[3][3]*self.mat[3][2]
        res[3][3] = other.mat[3][0]*self.mat[0][3] + other.mat[3][1]*self.mat[1][3] + other.mat[3][2]*self.mat[2][3] + other.mat[3][3]*self.mat[3][3]

        # 計算結果反映
        self.set(res)

    # 並行移動
    def Translate(self, tx, ty, tz):
        trans = [[1, 0, 0, tx], \
                 [0, 1, 0, ty], \
                 [0, 0, 1, tz], \
                 [0, 0, 0,  1]]

        self.mulL(trans)

    # X-Y-Z回転[deg] この回転だとズレるので本当はクォータニオンにしたい
    def RotateXYZ(self, rx, ry, rz):
        sx = math.sin(math.radians(rx))
        cx = math.cos(math.radians(rx))
        sy = math.sin(math.radians(ry))
        cy = math.cos(math.radians(ry))
        sz = math.sin(math.radians(rz))
        cz = math.cos(math.radians(rz))

        # 回転行列計算（X軸回転->Y軸回転->Z軸回転をひとまとめ）
        rot = [[cy*cz, sx*sy*cz-cx*sz, cx*sy*cz+sx*sz, 0], \
               [cy*sz, sx*sy*sz+cx*cz, cx*sy*sz-sx*cz, 0], \
               [  -sy,          sx*cy,          cx*cy, 0], \
               [    0,              0,              0, 1]]

        self.mulL(rot)

    # 座標変換 自分×変換対象(変換対象の形式は[x y z 1])
    def Transform(self, coordinate):
        # 結果格納用
        res = [0 for m in range(4)]

        # 変換実行
        res[0] = self.mat[0][0]*coordinate[0] + self.mat[0][1]*coordinate[1] + self.mat[0][2]*coordinate[2] + self.mat[0][3]*coordinate[3]
        res[1] = self.mat[1][0]*coordinate[0] + self.mat[1][1]*coordinate[1] + self.mat[1][2]*coordinate[2] + self.mat[1][3]*coordinate[3]
        res[2] = self.mat[2][0]*coordinate[0] + self.mat[2][1]*coordinate[1] + self.mat[2][2]*coordinate[2] + self.mat[2][3]*coordinate[3]
        res[3] = self.mat[3][0]*coordinate[0] + self.mat[3][1]*coordinate[1] + self.mat[3][2]*coordinate[2] + self.mat[3][3]*coordinate[3]

        return res
