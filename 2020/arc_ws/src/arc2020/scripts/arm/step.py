#!/usr/bin/python
# -*- coding: utf-8 -*-
# license removed for brevity

# pythonでROSのソフトウェアを記述するときにimportするモジュール
import rospy

# gpio制御用
import pigpio
import time

# モーター制御用
import mortor

# 自分で定義したmessageファイルから生成されたモジュール
from arc2020.msg import arm
from arc2020.msg import step

# 定数などの定義ファイルimport
from arm_consts import *

CYCLES = 60 

RET_ORGSW = 15


class Step(object):
    """
    全体の動作を制御するクラス
    """
#--------------------
# コンストラクタ
    def __init__(self):
        """
        コンストラクタ
        """
        # 受信作成
        self.sub_arm  = rospy.Subscriber('arm', arm, self.armCallback, queue_size=1)
        
        # 送信作成
        self.pub_step = rospy.Publisher('step', step, queue_size=100)
        # messageのインスタンスを作る
        self.msg_step = step()
        
        # 送信メッセージ初期化
        self.clearMsg()
        
        # MortorClass
        self.stmc = mortor.StepMortorClass(DEBUG_ARM, (PORT_HANDV_A, PORT_HANDV_B), (LIM_HANDV_MIN, LIM_HANDV_MAX))
        
        self.pi = pigpio.pi()
        self.pi.set_mode(RET_ORGSW, pigpio.INPUT)
        
        self.retorgcnt = 0
        
#--------------------
# 送信メッセージの初期化
    def clearMsg(self):
        """
        publishするメッセージのクリア
        """
        self.msg_step.stepstatus = 0
        self.msg_step.retorgsw = 0
        
#--------------------
# 受信コールバック
    def armCallback(self, arm_msg):
        """
        クライアントの受信コールバック
        """
        # メッセージ受信
        
        if arm_msg.drill_req :
            
            if self.retorgcnt == 0 :
                self.stmc.move_posinit_step()
                time.sleep(1)
                self.retorgcnt += 1
            else :
                # 処理 y軸移動⇒z軸降下⇒z軸上昇    
                handy = arm_msg.drill_width_value
                if handy < 0 :
                    handy = 0
                elif handy > 273 :
                    handy = 273
                
                self.msg_step.stepstatus = 1
                self.stmc.move_step(handy)  # y軸移動
                time.sleep(1)
                self.msg_step.stepstatus = 0
            
#--------------------
# メイン関数
    def main(self):
        
        if self.pi.read(RET_ORGSW) :
            self.msg_step.retorgsw = 0
        else :
            self.msg_step.retorgsw = 1
        
        # メッセージを発行する
        self.pub_step.publish(self.msg_step)
        
#--------------------
def step_py():
    # 初期化宣言 : このソフトウェアは"step_py_node"という名前
    rospy.init_node('step_py_node', anonymous=True)
    # インスタンスの作成 
    step = Step()
    # 処理周期の設定
    r = rospy.Rate(CYCLES)

    print("[step] start")
    # ctl +　Cで終了しない限りwhileループで処理し続ける
    while not rospy.is_shutdown():
        # メイン処理
        step.main()
        #
        r.sleep()

if __name__ == '__main__':
    try:
        step_py()

    except rospy.ROSInterruptException:
        print("[step] end")
