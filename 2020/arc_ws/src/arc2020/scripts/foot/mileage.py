#!/usr/bin/python
# -*- coding: utf-8 -*-
# license removed for brevity

# pythonでROSのソフトウェアを記述するときにimportするモジュール
import rospy

# gpio制御用
import pigpio

# 自分で定義したmessageファイルから生成されたモジュール
from arc2020.msg import mileage

ROT_ENCODE1 = 4
ROT_ENCODE2 = 14

CYCLES = 60 #処理周波数

PLS = 10 #ロータリエンコーダ1パルスの移動距離

# 定数などの定義ファイルimport


# class Led 定義
class Mileage(object):
    """
    スイッチの動作を制御するクラス
    """
#--------------------
# コンストラクタ
    def __init__(self):
        """
        コンストラクタ
        """
        # 送信作成
        self.pub_mileage = rospy.Publisher('mileage', mileage, queue_size=100)
        # messageのインスタンスを作る
        self.msg_mileage = mileage()

        #GPIOの初期設定
        self.pi = pigpio.pi()
        self.pi.set_mode(ROT_ENCODE1, pigpio.INPUT)
        self.pi.set_mode(ROT_ENCODE2, pigpio.INPUT)
        
        self.pi.callback(ROT_ENCODE1, pigpio.FALLING_EDGE, self.getRotationAB)
        self.pi.callback(ROT_ENCODE2, pigpio.FALLING_EDGE, self.getRotationBA)
        
        # 送信メッセージ初期化
        self.clearMsg()

#--------------------
# 送信メッセージの初期化
    def clearMsg(self):
        """
        publishするメッセージのクリア
        """
        self.msg_mileage.mileage = 0

#--------------------
    def main(self):
        # メッセージを発行する
        #print(self.msg_mileage.mileage)
        self.pub_mileage.publish(self.msg_mileage)

#--------------------
    def getRotationAB(self, gpio, level, tick):
        abcount = 0
        
        astate = self.pi.read(ROT_ENCODE1)
        bstate = self.pi.read(ROT_ENCODE2)
        
        if astate == 0 and bstate == 1 :
            abcount = 1
        
        self.msg_mileage.mileage += (abcount * PLS)

#--------------------
    def getRotationBA(self, gpio, level, tick):
        bacount = 0
        
        astate = self.pi.read(ROT_ENCODE1)
        bstate = self.pi.read(ROT_ENCODE2)
        
        if astate == 1 and bstate == 0 :
            bacount = -1
        
        self.msg_mileage.mileage += (bacount * PLS)

def mileage_py():
    # 初期化宣言 : このソフトウェアは"mileage_py_node"という名前
    rospy.init_node('mileage_py_node', anonymous=True)
    # インスタンスの作成 
    mileage = Mileage()
    # 処理周期の設定
    r = rospy.Rate(CYCLES)

    print("[mileage] start")
    # ctl +　Cで終了しない限りwhileループで処理し続ける
    while not rospy.is_shutdown():
        # メイン処理
        mileage.main()
        #
        r.sleep()

if __name__ == '__main__':
    try:
        mileage_py()

    except rospy.ROSInterruptException:
        print("[mileage] end")
