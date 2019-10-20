#!/usr/bin/env python
# -*- coding: utf-8 -*-

import pigpio
import rospy

from arc2019.msg import brain
from arc2019.msg import foot

# defined const
from params import DIRECTION
from brain_consts import CYCLES

# pin number
PIN_R1    = 24# PIN35 GPIO.24 Right1
PIN_R2    = 25# PIN37 GPIO.25 Right2
PIN_L1    = 27# GPIO.27 Left
PIN_L2    = 28# GPIO.28 
PIN = [PIN_R1,PIN_R2,PIN_L1,PIN_L2]

FREQ = 100   # PWM周波数　調整願います。
RANGE = 255  # PWM最大値　調整願います。

DUTY        = 1     # DUTY比　調整願います。
LOW         = 0     # 定数

RATE_LINEAR = 0.1     # 1mmあたりのモーター駆動時間（sec) 調整願います。
RATE_DEGREE = 1/180   # 1度あたりのモーター駆動時間（sec) 調整願います。

class FootClass(object):
    """
    足回りクラス
    """
    def __init__(self):
        """
        初期設定
        """
        self.pub = rospy.Publisher('foot',foot,queue_size=1)
        self.pi = pigpio.pi()
        for i in range(4):
            self.pi.set_mode(PIN[i], pigpio.OUTPUT)
            self.pi.set_PWM_frequency(PIN[i],FREQ)
            self.pi.set_PWM_range(PIN[i],RANGE) 

    def callback(self,foot):

        print"==============="
        print("dirreq = %d" % foot.foot_dirreq)
        print("movereq = %d" % foot.foot_movreq)
        print"==============="

        #前進
        if foot.foot_dirreq == DIRECTION.AHEAD:
            time = foot.foot_movreq * RATE_LINEAR 
            self.outputDIRECTION(DUTY, LOW, DUTY, LOW, time)# RIGHT: CW, LEFT: CW

        #後進
        elif foot.foot_dirreq == DIRECTION.BACK:
            time = foot.foot_movreq * RATE_LINEAR 
            self.outputDIRECTION(LOW, DUTY, LOW, DUTY, time)# RIGHT: CCW, LEFT : CCW

        #右旋回
        elif foot.foot_dirreq == DIRECTION.RIGHT:
            time = foot.foot_movreq * RATE_DEGREE
            self.outputDIRECTION(LOW, DUTY, DUTY, LOW, time)# RIGHT: CCW, LEFT: CW

        #左旋回
        elif foot.foot_dirreq == DIRECTION.LEFT:
            time = foot.foot_movreq * RATE_DEGREE
            self.outputDIRECTION(DUTY, LOW, LOW, DUTY, time)# RIGHT  CW, LEFT: CCW

        #停止
        else:
            time = 0
            self.outputDIRECTION(LOW, LOW, LOW, LOW, time)# RIGHT: ShortBreak, LEFT: ShortBreak


    def outputDIRECTION(self,R1, R2, L1, L2,time):     # 
        msg_foot = foot()
        #停止後、停止完了を送信
        msg_foot.is_foot_move = True
        self.pub.publish(msg_foot)
        """
        各PINへの出力
        """
        self.pi.set_PWM_dutycycle(PIN_R1,R1)
        self.pi.set_PWM_dutycycle(PIN_R2,R2)
        self.pi.set_PWM_dutycycle(PIN_L1,L1)
        self.pi.set_PWM_dutycycle(PIN_L2,L2)
        #設定時間sleep
        rospy.sleep(time)
        #sleep後停止
        for j in range(4):
            self.pi.set_PWM_dutycycle(PIN[j],LOW)
        #停止後、停止完了を送信
        msg_foot.is_foot_move = False
        self.pub.publish(msg_foot)

def foot_py():
    """
    Footのメイン
    """
    foot = FootClass()

    rospy.init_node('foot_py_node', anonymous=True)
    rrate = rospy.Rate(CYCLES)
    rospy.Subscriber('brain', brain, foot.callback, queue_size=1)
    print("start_foot")
    # ctl + Cで終了しない限りwhileループでpublishし続ける
    while not rospy.is_shutdown():
        rrate.sleep()
#end abh_py
if __name__ == '__main__':
    foot_py()
