#!/usr/bin/python
# -*- coding: utf-8 -*-
# license removed for brevity

# pythonでROSのソフトウェアを記述するときにimportするモジュール
import rospy

# gpio制御用
import pigpio

# 標準ライブラリ
import math

# 自分で定義したmessageファイルから生成されたモジュール
from arc2019.msg import client
from arc2019.msg import brain
from arc2019.msg import eye
from arc2019.msg import arm
from arc2019.msg import foot

# 定数などの定義ファイルimport
from brain_consts import CYCLES
from params import Mode, TARGET, CAMERA

#
HZ_PUBLISH        = 60  #Pubrishの周期
HZ_MODE           = 5
SPEED_STEP        = 64
#
STICK_RIGHT_H     = 2   #右スティック水平 
STICK_RIGHT_V     = 5   #右スティック垂直 
STICK_LEFT_H      = 0   #左スティック水平 
STICK_LEFT_V      = 1   #左スティック垂直 
BUTTON_R1         = 18  #R1 
BUTTON_R2         = 4   #R2 
BUTTON_L1         = 17  #L1 
BUTTON_L2         = 3   #L2 
BUTTON_O          = 15  #Oボタン（サークル） 
BUTTON_X          = 14  #xボタン (クロス） 
BUTTON_DELTA      = 16  #△ボタン（デルタ） 
BUTTON_SQUARE     = 13  #□ボタン（スクエア） 
BUTTON_SHARE      = 21  #SHARE 
BUTTON_OPTION     = 22  #OPTION 
CROSS_H           = 9   #十字キー水平 左1 右-1
CROSS_V           = 10  #十字キー垂直 上1 下-1

#
INDEX_DIRECTION_H = STICK_RIGHT_H  
INDEX_DIRECTION_V = STICK_RIGHT_V 
INDEX_SPEED_L     = STICK_LEFT_V 
INDEX_SPEED_R     = STICK_RIGHT_V  
INDEX_STRIKE_ON   = BUTTON_O
INDEX_STRIKE_OFF  = BUTTON_X
INDEX_GRUB_ON     = BUTTON_O 
INDEX_GRUB_OFF    = BUTTON_X
INDEX_HOME        = BUTTON_DELTA 
INDEX_STORE       = BUTTON_L1
INDEX_TILT        = CROSS_V 
INDEX_UP          = BUTTON_R1
INDEX_DOWN        = BUTTON_R2 
INDEX_RELEASE     = BUTTON_SHARE
INDEX_MODE        = BUTTON_OPTION
INDEX_MAX         = BUTTON_OPTION
#
MAX = 1
MIN = -1

#
ON  = 1
OFF = 0 

# 3次元座標の変換用
# 高速化のためfor文を使わずベタ書き
#
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


class Brain(object):
    def __init__(self):
        self.cyclecount = 0
        # 受信作成
        self.sub_client  = rospy.Subscriber('client', client, self.clientCallback, queue_size=1)
        self.sub_eye  = rospy.Subscriber('client', eye, self.eyeCallback, queue_size=1)
        self.sub_arm  = rospy.Subscriber('arm', arm, self.armCallback, queue_size=1)
        self.sub_foot = rospy.Subscriber('foot', foot, self.footCallback, queue_size=1)
        # 送信作成
        self.pub_brain = rospy.Publisher('brain', brain, queue_size=100)
        # messageのインスタンスを作る
        self.rmsg_client = client()
        self.msg_brain = brain()
        #デフォルト
        self.mode = Mode.INIT
        self.target = TARGET.UNKNOWN

        #カメラ→アームへの変換行列作成
        self.maincam = Transform3D()
        self.subcam = Transform3D()
        self.pollcam = Transform3D()

    def clearMsg(self):
        #for all
        self.msg_brain.mode_id = Mode.UNKNOWN
        self.msg_brain.target_id = TARGET.UNKNOWN
        #for arm
        self.msg_brain.hand_req     = 0
        self.msg_brain.pluck_req    = 0
        self.msg_brain.elbow_req    = 0
        self.msg_brain.should_req   = 0
        self.msg_brain.handx_req    = 0
        self.msg_brain.handy_req    = 0
        self.msg_brain.handz_req    = 0
        self.msg_brain.twistx_req   = 0
        self.msg_brain.twistz_req   = 0

    def initialize(self):
        clearMsg()
        self.msg_brain.mode_id = Mode.INIT

        # Clientから動作モード&ターゲットを受けるまで1秒間隔でpublish
        if self.rmsg_client.mode == Mode.UNKNOWN | self.rmsg_client.tartget == TARGET.UNKNOWN:
            if self.cyclecount == 0
                self.pub_brain.publish(self.msg_brain)
            else:
                pass
        else:
            self.mode = self.rmsg_client.mode
            self.target = self.rmsg_client.target

    def clientCallback(self, client_msg):
        self.rmsg_client = client_msg

    def eyeCallback(self, eye_msg):
        if eye_msg.camera_id == CAMERA.MAIN:
            #トマト、雑草、主枝
            pass    #TO DO
        elif eye_msg.camera_id == CAMERA.SUB:
            #脇芽
            pass    #TO DO
        elif eye_msg.camera_id == CAMERA.POLL:
            #ポール
            pass    #TO DO

         
    def main(self):
        if self.mode == Mode.INIT:
            initialize()
        else
            #メインシーケンス
            pass

        # サイクル数カウント
        self.cyclecount += 1
        if self.cyclecount == CYCLES:
            self.cyclecount = 0


def brain_py():
    # 初期化宣言 : このソフトウェアは"brain_py_node"という名前
    rospy.init_node('brain_py_node', anonymous=True)
    # インスタンスの作成 
    brain = Brain()
    # 処理周期の設定
    r = rospy.Rate(CYCLES)

    print"start brain"
    # ctl +　Cで終了しない限りwhileループで処理し続ける
    while not rospy.is_shutdown():
        # メイン処理
        brain.main()
        #
        r.sleep()

if __name__ == '__main__':
    try:
        brain_py()

    except rospy.ROSInterruptException: pass

