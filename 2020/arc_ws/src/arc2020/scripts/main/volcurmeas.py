#!/usr/bin/python
# -*- coding: utf-8 -*-
# license removed for brevity

# pythonでROSのソフトウェアを記述するときにimportするモジュール
import rospy

"""
電流電圧測定IC
"""

from subprocess import call
import time
import smbus
#10.10.8 wata
#多分このmortorをimportする必要ないと思われる。
##import mortor

from volcurmeas_consts import *


# 自分で定義したmessageファイルから生成されたモジュール
from arc2020.msg import volcurmeas

CYCLES = 60 #処理周波数
# 定数などの定義ファイルimport

v_aves = []
i_aves = []

# class  定義
class VOLCURMEAS_DEV(object):
    """
    電圧電流測定ICの動作を制御するクラス
    """
#--------------------
# コンストラクタ
    def __init__(self):
        """
        コンストラクタ
        """
        self.cyclecount = 0
        # 受信作成
        #self.sub_client  = rospy.Subscriber('main', main, self.mainCallback, queue_size=1)
        # 送信作成
        self.pub_volcurmeas = rospy.Publisher('volcurmeas', volcurmeas, queue_size=100)
        # messageのインスタンスを作る
        self.msg_volcurmeas = volcurmeas()

        #電流電圧測定ICの初期設定やら
        self.is_enable = False
        print('init_g')
        try:

            print('try')
#10.10.8 wata
#多分ここはいらないはず。
#            self.smc = mortor.ServoMortorClass(False)
            self.i2c = smbus.SMBus(1)
            print('SMBus')
            # ICの設定
#10.10.8 wata
#この設定はデフォルト設定で問題なし。
#            setdata = (SET_RST << 12) + (SET_AVG << 9) + \
#                (SET_VBUSCT << 6) + (SET_VSHCT << 3) + SET_MODE
#            setdata = ((setdata << 8) & 0xFF00) + (setdata >> 8)
#            self.i2c.write_word_data(I2C_INA226, ADDR_S, setdata)

            # キャリブレーションレジスタの設定
            # 0.00512/((0.0015[mΩ])*0.001)
            #setdata = int(0.00512/((BATT_R)*0.001))
            setdata = 0x0800
            setdata = ((setdata << 8) & 0xFF00) + (setdata >> 8)
            self.i2c.write_word_data(I2C_INA226, ADDR_R, setdata)
            print('init')
            self.is_enable = True

# 1回目は変な値をとるときが多いので...
#10.10.8 wata
# 多分いらないはず。。。。
#            self.read_v()
#            self.read_i()
        except IOError:
            self.is_enable = False
            print('error')

        finally:
            self.is_battlow = False
            self.cnt_battlow = 0
            self.v_ave = 0
            self.i_ave = 0
            self.i_sgm = 0

            print('test')
    # end __init__

    def read_v(self):
        """
        電圧読み取り
        """
        volt = 0.0
        if self.is_enable:
            word = self.i2c.read_word_data(I2C_INA226, ADDR_V) & 0xFFFF
            result = ((word << 8) & 0xFF00) + (word >> 8)
            volt = result * 1.25 / 1000

        return volt
    # end read_v

    def read_i(self):
        """
        電流読み取り
        """
        curr = 0.0
        if self.is_enable:
            word = self.i2c.read_word_data(I2C_INA226, ADDR_I) & 0xFFFF
#10.10.8 wata            curr = ((word << 8) & 0xFF00) + (word >> 8)
            result = ((word << 8) & 0xFF00) + (word >> 8)
            curr = result / 1000.0000
        return curr
    # end read_i

    def read_b(self):
        """
        バッテリー残量読み取り
        """
        batt = 0.0
        if self.is_enable:
#10.10.8 wata            batt_v = self.read_i()
            batt_v = self.read_v()
            batt = ((batt_v - BATT_VLOW) / (BATT_VMAX - BATT_VLOW)) * 100
        return batt

    def read_vi_loop(self):
        """
        電流電圧読み取りループ
        """
#        v_aves = []
#        i_aves = []
#        while self.is_enable:
        self.v_now = self.read_v()
#        v_aves.append(v_now)
#        if len(v_aves) > 100:
#            del v_aves[0]
#        self.v_ave = sum(v_aves) / len(v_aves)

        self.i_now = self.read_i()
#        self.i_sgm += i_now
#        i_aves.append(i_now)
#        if len(i_aves) > 100:
#            del i_aves[0]
#        self.i_ave = sum(i_aves) / len(i_aves)

#        print "NOW={:.2f}[V]\t".format(v_now) + "AVE={:.2f}[V]\t".format(self.v_ave) + \
#            "NOW={:.4f}[A]\t".format(i_now) + "AVE={:.4f}[A]\t".format(self.i_ave) + \
#            "SGM={:.4f}[Asec]\t".format(self.i_sgm)
#
#        if self.v_ave < BATT_VLOW:
#            self.cnt_battlow += 1
#            print "電圧低下(" + str(self.cnt_battlow) + ")"
#        elif self.i_ave > BATT_IHI:
#            # self.cnt_battlow += 1
#            print "電流異常(" + str(self.cnt_battlow) + ")"
#        else:
#            self.cnt_battlow = 0
#
#        if self.cnt_battlow > BATT_ERR:
#            self.endfnc()
#
#        else:
#            pass

        time.sleep(1)
    # end read_vi_loop

    def endfnc(self):
        """
        終了処理
        """
        self.smc.endfnc()
        print "1分後にシャットダウンします"
        call('sudo shutdown -h 1', shell=True)
        self.is_enable = False
    # end endfnc

#--------------------
# 受信コールバック
#    def mainCallback(self, main_msg):
#        """
#        mainの受信コールバック
#        """
#        #print("led a" + str(main_msg.led_a_value)) 
#        self.pi.set_PWM_dutycycle(PIN_A,main_msg.led_a_value)
#        self.pi.set_PWM_dutycycle(PIN_B,main_msg.led_b_value)
#--------------------
    def main(self):

        self.read_vi_loop()

        # メッセージを発行する
        self.msg_volcurmeas.volt = self.v_now
        self.msg_volcurmeas.cur = self.i_now
        self.pub_volcurmeas.publish(self.msg_volcurmeas)

        print(self.v_now)
        print(self.i_now)
def volcurmeas_py():
    # 初期化宣言 : このソフトウェアは"led_py_node"という名前
    rospy.init_node('volcurmeas_py_node', anonymous=True)
    # インスタンスの作成 
    volcurmeas_dev = VOLCURMEAS_DEV()
    # 処理周期の設定
    r = rospy.Rate(CYCLES)

    print("[volcurmeas] start")
    # ctl +　Cで終了しない限りwhileループで処理し続ける
    while not rospy.is_shutdown():
        # メイン処理
        volcurmeas_dev.main()
        #
        r.sleep()

if __name__ == '__main__':
    try:
        volcurmeas_py()

    except rospy.ROSInterruptException:
        print("[volcurmeas] end")
