#!/usr/bin/python
# -*- coding: utf-8 -*-
# license removed for brevity

# pythonでROSのソフトウェアを記述するときにimportするモジュール
import rospy

# gpio制御用
import pigpio

# 自作ライブラリ
from .mylib.Vec3D import Vec3D
from .mylib.Transform3D import Transform3D

# 自分で定義したmessageファイルから生成されたモジュール
from arc2019.msg import client
from arc2019.msg import brain
from arc2019.msg import eye
from arc2019.msg import arm
from arc2019.msg import foot

# 定数などの定義ファイルimport
from brain_consts import CYCLES
from params import Mode, TARGET, CAMERA

# #
# STICK_RIGHT_H     = 2   #右スティック水平 
# STICK_RIGHT_V     = 5   #右スティック垂直 
# STICK_LEFT_H      = 0   #左スティック水平 
# STICK_LEFT_V      = 1   #左スティック垂直 
# BUTTON_R1         = 18  #R1 
# BUTTON_R2         = 4   #R2 
# BUTTON_L1         = 17  #L1 
# BUTTON_L2         = 3   #L2 
# BUTTON_O          = 15  #Oボタン（サークル） 
# BUTTON_X          = 14  #xボタン (クロス） 
# BUTTON_DELTA      = 16  #△ボタン（デルタ） 
# BUTTON_SQUARE     = 13  #□ボタン（スクエア） 
# BUTTON_SHARE      = 21  #SHARE 
# BUTTON_OPTION     = 22  #OPTION 
# CROSS_H           = 9   #十字キー水平 左1 右-1
# CROSS_V           = 10  #十字キー垂直 上1 下-1

# #
# INDEX_DIRECTION_H = STICK_RIGHT_H  
# INDEX_DIRECTION_V = STICK_RIGHT_V 
# INDEX_SPEED_L     = STICK_LEFT_V 
# INDEX_SPEED_R     = STICK_RIGHT_V  
# INDEX_STRIKE_ON   = BUTTON_O
# INDEX_STRIKE_OFF  = BUTTON_X
# INDEX_GRUB_ON     = BUTTON_O 
# INDEX_GRUB_OFF    = BUTTON_X
# INDEX_HOME        = BUTTON_DELTA 
# INDEX_STORE       = BUTTON_L1
# INDEX_TILT        = CROSS_V 
# INDEX_UP          = BUTTON_R1
# INDEX_DOWN        = BUTTON_R2 
# INDEX_RELEASE     = BUTTON_SHARE
# INDEX_MODE        = BUTTON_OPTION
# INDEX_MAX         = BUTTON_OPTION
# #
# MAX = 1
# MIN = -1

# #
# ON  = 1
# OFF = 0 

class Brain(object):
#--------------------
# コンストラクタ
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
        self.msg_brain = brain()
        #デフォルト
        self.mode = Mode.INIT
        self.target = TARGET.UNKNOWN

        self.maintgt = Vec3D()
        self.subtgt = Vec3D()
        self.poll = Vec3D()

        #カメラ→アームへの変換行列作成
        #事前にロボットのズレを測定して適用する
        self.maincam = Transform3D()
        self.maincam.set([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])  #座標軸入れ替え・方向反転
        self.maincam.Translate(0, 0, 0) #ズレ補正[mm]
        self.maincam.RotateXYZ(0, 0, 0) #ズレ補正[deg]

        self.subcam = Transform3D()
        self.subcam.set([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])   #座標軸入れ替え・方向反転
        self.subcam.Translate(0, 0, 0)  #ズレ補正[mm]
        self.subcam.RotateXYZ(0, 0, 0)  #ズレ補正[deg]

        self.pollcam = Transform3D()
        self.pollcam.set([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])  #座標軸入れ替え・方向反転
        self.pollcam.Translate(0, 0, 0) #ズレ補正[mm]
        self.pollcam.RotateXYZ(0, 0, 0) #ズレ補正[deg]
#--------------------

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
        self.clearMsg()
        self.msg_brain.mode_id = Mode.INIT

        # Clientから動作モード&ターゲットを受けるまで1秒間隔でpublish
        if self.mode == Mode.UNKNOWN | self.tartget == TARGET.UNKNOWN:
            if self.cyclecount == 0:
                self.pub_brain.publish(self.msg_brain)
            else:
                pass
        else:
            pass

#--------------------
# 受信コールバック
    def clientCallback(self, client_msg):
        self.mode = client_msg.mode
        self.target = client_msg.target

    def eyeCallback(self, eye_msg):
        if eye_msg.target_find == True:
            coord = [eye_msg.target_x, eye_msg.target_y, eye_msg.target_z, 1]
            
            if eye_msg.camera_id == CAMERA.MAIN:
                #トマト、雑草、主枝
                res = self.maincam.Transform(coord)
                self.maintgt.x = res[0]
                self.maintgt.y = res[1]
                self.maintgt.z = res[2]
            elif eye_msg.camera_id == CAMERA.SUB:
                #脇芽
                res = self.subcam.Transform(coord)
                self.maintgt.x = res[0]
                self.maintgt.y = res[1]
                self.maintgt.z = res[2]
            elif eye_msg.camera_id == CAMERA.POLL:
                #ポール
                res = self.subcam.Transform(coord)
                self.poll.x = res[0]
                self.poll.y = res[1]
                self.poll.z = res[2]s
        else:
            pass
     
    def armCallback(self, arm_msg):
        self.is_arm_move = arm_msg.is_arm_move

    def footCallback(self, foot_msg):
        self.is_foot_move = foot_msg.is_foot_move
#--------------------


    def main(self):
        if self.mode == Mode.INIT:
            self.initialize()
        elif self.mode == Mode.MANUAL:
            #Todo 手動シーケンス
        elif self.mode == Mode.AUTO:
            if self.target == TARGET.GRASS:
            #Todo 草刈りシーケンス
            if self.target == TARGET.TOMATO:
            #Todo 収穫シーケンス
            if self.target == TARGET.SIDE_SPROUT:
            #Todo 芽かきシーケンス
        else:
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

    print("start brain")
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
