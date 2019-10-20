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

FREQ = 10*1000  # PWM周波数　調整願います。
DUTY = 50       # DUTY比 (%)
RANGE = 255     # PWM最大値　調整願います。

HIGH        = 1     # HIGH比　調整願います。
LOW         = 0     # 定数

RATE_LINEAR = 0.1*1000     # 1mmあたりのモーター駆動時間（msec) 調整願います。
RATE_DEGREE = (1/180)*1000   # 1度あたりのモーター駆動時間（msec) 調整願います。

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
          #  self.pi.set_PWM_frequency(PIN[i],FREQ)
          #  self.pi.set_PWM_range(PIN[i],RANGE) 

    def callback(self,foot):

        print"==============="
        print("dirreq = %d" % foot.foot_dirreq)
        print("movereq = %d" % foot.foot_movreq)
        print"==============="

        #前進
        if foot.foot_dirreq == DIRECTION.AHEAD:
            time = foot.foot_movreq * RATE_LINEAR 
            self.outputDIRECTION(HIGH, LOW, HIGH, LOW, time)# RIGHT: CW, LEFT: CW

        #後進
        elif foot.foot_dirreq == DIRECTION.BACK:
            time = foot.foot_movreq * RATE_LINEAR 
            self.outputDIRECTION(LOW, HIGH, LOW, HIGH, time)# RIGHT: CCW, LEFT : CCW

        #右旋回
        elif foot.foot_dirreq == DIRECTION.RIGHT:
            time = foot.foot_movreq * RATE_DEGREE
            self.outputDIRECTION(LOW, HIGH, HIGH, LOW, time)# RIGHT: CCW, LEFT: CW

        #左旋回
        elif foot.foot_dirreq == DIRECTION.LEFT:
            time = foot.foot_movreq * RATE_DEGREE
            self.outputDIRECTION(HIGH, LOW, LOW, HIGH, time)# RIGHT  CW, LEFT: CCW

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
        duration = 0
        start = rospy.Time.now()
        print ("now %f sec, %f nsec" % (start.secs,start.nsecs))
        while duration <  time:
            self.pi.write(PIN_R1,R1)
            self.pi.write(PIN_R2,R2)
            self.pi.write(PIN_L1,L1)
            self.pi.write(PIN_L2,L2)
            #設定時間sleep
            rospy.sleep((1/FREQ)*DUTY/100)
            #sleep後停止
            for j in range(4):
                self.pi.write(PIN[j],LOW)
            rospy.sleep((1/FREQ)*(1-DUTY/100))
            now = rospy.Time.now()
            
            sec = (now.secs - start.secs)*1000
            duration = sec + (now.nsecs - start.nsecs)/(1000*1000)
            #print ("duraiton %f" % duration)
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
