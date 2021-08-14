#!/usr/bin/python
# -*- coding: utf-8 -*-
# license removed for brevity

# pythonでROSのソフトウェアを記述するときにimportするモジュール
import rospy

# gpio制御用
import pigpio

# 自分で定義したmessageファイルから生成されたモジュール
from arc2020.msg import foot


from dc_motor import DCMotor
from hcsr04 import HCSR04Class
from i2c_PCF8574 import I2CPCF8574
from i2c_BMX055 import I2CBMX055

#import GPIOPIN
from GPIO import GPIOPIN
# class Led 定義

CYCLES = 60

class FootDebug(object):
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
        self.pub = rospy.Publisher('foot_sensor', foot, queue_size=100)
        # messageのインスタンスを作る
        self.msg_foot = foot()
        
        for i in range(2):
            self.msg_foot.sonor.append(0)
        for i in range(8):
            self.msg_foot.line.append(0)
        for i in range(3):
            self.msg_foot.accel.append(0)
            self.msg_foot.gyro.append(0)
            self.msg_foot.mag.append(0)
        
        # DC Motor用のインスタンス作成
        self.motor_r =  DCMotor(GPIOPIN.DC_MOTOR_A1.value,GPIOPIN.DC_MOTOR_A2.value)
        self.motor_l =  DCMotor(GPIOPIN.DC_MOTOR_B1.value,GPIOPIN.DC_MOTOR_B2.value)

        # Sonor用のインスタンス作成
        self.sonor = []
        port = (GPIOPIN.SONAR_TRIG1.value,GPIOPIN.SONAR_PULS.value)
        self.sonor.append(HCSR04Class(False,port))

        port = (GPIOPIN.SONAR_TRIG2.value,GPIOPIN.SONAR_PULS.value)
        self.sonor.append(HCSR04Class(False,port))
        self.sonor[0].start()
        self.sonor[1].start()
        # line tracer用のインスタンス作成
        self.line = I2CPCF8574()

        # 9D sensor 用のインスタンス作成
        self._9dsensor = I2CBMX055()

        # Subscriber登録
        rospy.Subscriber('foot_debug', foot, self.callback, queue_size=1)
        
        # 送信メッセージ初期化
        #self.clearMsg()
    def callback(self,msg):
        self.motor_r.changeDuty(msg.motor_r)
        self.motor_l.changeDuty(msg.motor_l)

#--------------------
    def main(self):
        for i in range(2):
            self.msg_foot.sonor[i] = self.sonor[i].read()
        #for i in range(8):
        self.msg_foot.line = self.line.read()
        self.msg_foot.accel = self._9dsensor.read_accel()
        self.msg_foot.gyro = self._9dsensor.read_gyro()
        self.msg_foot.mag = self._9dsensor.read_mag()

        # メッセージを発行する
        #print("sonor1 = " + str(self.msg_foot.sonor[0]))
        # print("sonor2 = " + str(self.msg_foot.sonor_2))
        self.pub.publish(self.msg_foot)


def foot_debug_py():
    # 初期化宣言 : このソフトウェアは"mileage_py_node"という名前
    rospy.init_node('foot_debug_py_node', anonymous=True)
    # インスタンスの作成 
    foot = FootDebug()
    # 処理周期の設定
    r = rospy.Rate(CYCLES)

    print("[foot_debug] start")
    # ctl +　Cで終了しない限りwhileループで処理し続ける
    while not rospy.is_shutdown():
        # メイン処理
        foot.main()
        #
        r.sleep()

if __name__ == '__main__':
    try:
        foot_debug_py()

    except rospy.ROSInterruptException:
        print("[foot_debug] end")
