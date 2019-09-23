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
from brain_consts import CYCLES, WAIT
from params import Mode, TARGET, CAMERA, DIRECTION


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
        self.mode = self.rx_mode = Mode.INIT
        self.target = self.rx_target = TARGET.UNKNOWN

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
        self.msg_brain.mode_id = self.mode
        self.msg_brain.target_id = self.target
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
        #for foot
        self.msg_brain.foot_dirreq  = 0
        self.msg_brain.foot_movreq  = 0
#--------------------
    def OnOperation(self):
        if self.is_arm_move | self.is_foot_move:
            # 手・足いずれかが駆動中
            self.waitformove = False
            self.trans_time  = CYCLES * WAIT
            return True
        elif self.waitformove:
            # 駆動指令をpublish済みだが、手・足がまだ動作中ではない中間状態
            return True
        else:
            return False

    def OnTransition(self):
        if self.trans_time > 0:
            # 手・足駆動終了後に一定時間Waitを挟む（カメラのブレを抑える目的）
            self.trans_time -= 1
            return True
        else:
            return False

    def DoesFindAny(self):
        if self.maintgt_find | self.subtgt_find | self.poll_find:
            return True
        else:
            return False
#--------------------
# 受信コールバック
    def clientCallback(self, client_msg):
        self.rx_mode = client_msg.mode
        self.rx_target = client_msg.target

    def eyeCallback(self, eye_msg):
        if OnOperation():
            #駆動中は取得しない
            pass
        elif eye_msg.target_find == True:
            coord = [eye_msg.target_x, eye_msg.target_y, eye_msg.target_z, 1]
            
            if eye_msg.camera_id == CAMERA.MAIN:
                #トマト、雑草
                res = self.maincam.Transform(coord)
                self.maintgt.x = res[0]
                self.maintgt.y = res[1]
                self.maintgt.z = res[2]
                self.maintgt_find = True
            elif eye_msg.camera_id == CAMERA.SUB:
                #脇芽
                res = self.subcam.Transform(coord)
                self.subtgt.x = res[0]
                self.subtgt.y = res[1]
                self.subtgt.z = res[2]
                self.subtgt_find = True
            elif eye_msg.camera_id == CAMERA.POLL:
                #ポール
                res = self.subcam.Transform(coord)
                self.poll.x = res[0]
                self.poll.y = res[1]
                self.poll.z = res[2]
                self.poll_find = True
     
    def armCallback(self, arm_msg):
        self.is_arm_move = arm_msg.is_arm_move

    def footCallback(self, foot_msg):
        self.is_foot_move = foot_msg.is_foot_move

#--------------------
# eye
    def IsTargetCenter(self,pos_y):
        # 対象がカメラ中央付近にあるかチェックする
        # (中央付近にないと、奥行き方向の精度が下がる)

        # pos_y : Vec3D.y
        if -CENTER_THRESH <= pos_y | pos_y <= CENTER_THRESH:
            return True
        else:
            return False

#--------------------
# arm
    def DriveHand(self, tgtpos):
        # tgtpos : Vec3D
        self.msg_brain.handx_req = tgtpos.x
        self.msg_brain.handy_req = tgtpos.y
        self.msg_brain.handz_req = tgtpos.z

        self.pub_brain.publish(self.msg_brain)
        self.waitformove = True
        self.clearMsg()

    def DriveTwist(self, tgtpos):
        # tgtpos : Vec3D
        self.msg_brain.twistx_req = tgtpos.x
        self.msg_brain.twistz_req = tgtpos.z
        
        self.pub_brain.publish(self.msg_brain)
        self.waitformove = True
        self.clearMsg()

#--------------------
# foot
    def GoAhead(self,mm):
        # 前進
        self.msg_brain.foot_dirreq = DIRECTION.AHEAD
        self.msg_brain.foot_movreq = mm

        self.pub_brain.publish(self.msg_brain)
        self.waitformove = True
        self.clearMsg()

    def GoBack(self,mm):
        # 後退
        self.msg_brain.foot_dirreq = DIRECTION.BACK
        self.msg_brain.foot_movreq = mm

        self.pub_brain.publish(self.msg_brain)
        self.waitformove = True
        self.clearMsg()
    
    def RotateRight(self,deg):
        # 右旋回
        self.msg_brain.foot_dirreq = DIRECTION.RIGHT
        self.msg_brain.foot_movreq = deg

        self.pub_brain.publish(self.msg_brain)
        self.waitformove = True
        self.clearMsg()

    def RotateLeft(self,deg):
        # 左旋回
        self.msg_brain.foot_dirreq = DIRECTION.LEFT
        self.msg_brain.foot_movreq = deg

        self.pub_brain.publish(self.msg_brain)
        self.waitformove = True
        self.clearMsg()

    def AdjustingMove(self, mm):
        if mm >= 0:
            GoAhead(mm)
        else:
            GoBack(mm)
#--------------------
    def initialize(self):
        self.clearMsg()

        # Clientから動作モード&ターゲットを受けるまで1秒間隔でpublish
        if self.rx_mode == Mode.INIT | self.rx_tartget == TARGET.UNKNOWN:
            if self.cyclecount == 0:
                self.pub_brain.publish(self.msg_brain)
            else:
                pass
        else:
            # モード/ターゲット更新 & 通知
            self.msg_brain.mode_id = self.mode = self.rx_mode
            self.msg_brain.target_id = self.target = self.rx_target
            self.pub_brain.publish(self.msg_brain)

    def main(self):
        if self.mode == Mode.INIT:
            self.initialize()
        elif self.mode == Mode.MANUAL:
            pass #Todo 手動シーケンス
        elif self.mode == Mode.AUTO:
            if OnOperation():
                # 手・足いずれかが駆動中は何もしない
                pass
            elif OnTransition():
                # 手・足駆動終了後に一定時間Wait（カメラのブレが収まるのを待つ）
                pass
            elif DoesFindAny():
                # 検出対象を発見

                #Todo 畝に寄る？

                if self.maintgt_find:
                    if IsTargetCenter(self.maintgt.y):
                        DriveHand(self.maintgt)
                    else:
                        AdjustingMove(-self.maintgt.y/2.0)
                elif self.subtgt_find:
                    if IsTargetCenter(self.subtgt.y):
                        DriveTwist(self.subtgt)
                    else:
                        AdjustingMove(-self.subtgt.y/2.0)
                elif self.poll_find:
                    pass #Todo ポール検出時動作
            else:
                # 何も見つからない場合は前進
                GoAhead(DEFAULT_MOV)
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

    except rospy.ROSInterruptException:
        print("end brain")
