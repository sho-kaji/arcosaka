#!/usr/bin/python
# -*- coding: utf-8 -*-
# license removed for brevity

# pythonでROSのソフトウェアを記述するときにimportするモジュール
import rospy

# gpio制御用
import pigpio

# 自分で定義したmessageファイルから生成されたモジュール
from arc2020.msg import foot
from arc2020.msg import main

# サブモジュールのimport
from dc_motor import DCMotor
from hcsr04 import HCSR04Class
from i2c_PCF8574 import I2CPCF8574
from i2c_BMX055 import I2CBMX055
from mileage import Mileage
from pid_control import PIDControl

#import GPIOPIN
from GPIO import GPIOPIN
# class Led 定義

CYCLES = 60

# OPERATION
OPE_START = 0
OPE_PAUSE = 1
OPE_STOP  = 2

# MOVE_STATUS
MOVE_INPROGRESS = 0
MOVE_COMPLETE = 1
MOVE_IDLE = 2
MOVE_INPROGRESS_PAUSE = 3

# MOTOR_VALUE
MOTOR_AHEAD = 10
MOTOR_STOP = 0

class FootMain(object):
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
        self.msg_foot.movestatus = MOVE_COMPLETE
        self.msg_main = main()
        # センサ値格納用のメンバ変数定義
        self.sonor_val = []
        for i in range(2):
            self.sonor_val.append(0)
        
        self.line_val = []
        for i in range(8):
            self.line_val.append(0)
        
        self.accel_val = []
        self.gyro_val = []
        self.mag_val = []
        for i in range(3):
            self.accel_val.append(0)
            self.gyro_val.append(0)
            self.mag_val.append(0)
        
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

        # rotary encorder用のinstance作成
        self.mileage = Mileage()

        # pid control instance
        self.pid = PIDControl()
        
        self.operation = OPE_PAUSE
        self.move_status = MOVE_IDLE
        self.x_index = -1
        # Subscriber登録
        rospy.Subscriber('foot_debug', foot, self.callback, queue_size=1)
        rospy.Subscriber('from_main', main, self.callback, queue_size=1)
#--------------------      
        # 送信メッセージ初期化
        #self.clearMsg()
    def callback(self,msg):
        self.operation = msg.foot_operation
        self.x_cordinate = msg.foot_x_cordinate
        self.mode = msg.foot_mode
        self.motor_l_value = msg.foot_motor_l
        self.motor_r_value = msg.foot_motor_r
        #self.motor_r.changeDuty(msg.motor_r)
        #self.motor_l.changeDuty(msg.motor_l)

#--------------------
    def read_sensors(self):
        for i in range(2):
            self.sonor_val[i] = self.sonor[i].read()
        #for i in range(8):
        self.line_val = self.line.read()
        self.accel_val = self._9dsensor.read_accel()
        self.gyro_val = self._9dsensor.read_gyro()
        self.mag_val = self._9dsensor.read_mag()
        self.mileage_val = self.mileage.read_mileage()

#--------------------
    def change_move_status(self):
        # IDLE
        if self.move_status == MOVE_IDLE:
            # -> INPROGRESS
            if self.operation == OPE_START:
                self.move_status = MOVE_INPROGRESS
                self.x_index = self.x_index + 1
        
        # INPROGRESS
        elif self.move_status == MOVE_INPROGRESS:
            # -> COMPLETE
            if self.x_cordinate[self.x_index] <= self.mileage_val:
                self.move_status = MOVE_COMPLETE
                if self.x_index == len(self.x_cordinate) - 1:
                    self.x_index = -1
            # -> INPROGRESS_PAUSE
            elif self.operation == OPE_PAUSE:
                self.move_status = MOVE_INPROGRESS_PAUSE
        
        # INPROGRESS_PAUSE
        elif self.move_status == MOVE_INPROGRESS_PAUSE:
            # -> INPROGRESS
            if self.operation == OPE_START:
                self.move_status = MOVE_INPROGRESS
        
        # COMPLETE    
        elif self.move_status == MOVE_COMPLETE:
            # -> IDLE
            if self.operation == OPE_PAUSE:
                self.move_status = MOVE_IDLE
        
        # STOPを受けたらindexを-1にしてIDLEにする
        if self.operation == OPE_STOP:
            self.move_status = MOVE_IDLE
            self.x_index = -1

        return
                        
#--------------------
    def return_move_status(self):
        prev_move_status = self.msg_foot.movestatus
        # COMPLETE
        if (self.move_status == MOVE_IDLE 
        or self.move_status == MOVE_COMPLETE) :
            self.msg_foot.movestatus = MOVE_COMPLETE
        # INPROGRESS
        elif (self.move_status == MOVE_INPROGRESS
        or self.move_status == MOVE_INPROGRESS_PAUSE) :
            self.msg_foot.movestatus = MOVE_INPROGRESS
        # 状態が変化すればmainへ通知する
        if prev_move_status != self.msg_foot.movestatus:
            self.pub.publish(self.msg_foot)
        return
         
#--------------------
    def get_operation_bias(self):
        bias = 0
        return bias

#--------------------
    def move(self,bias):
        # センサ読み取り
        self.read_sensors()
        # bias計算
        bias_line = self.pid.get_bias_line_trace(self.line_val)
        bias_range = self.pid.get_bias_side_range(self.sonor_val[0],self.sonor_val[1])
        print( "bias line:" + str(bias_line) + " range:" + str(bias_range))
        bias = bias_line + bias_range
        # DCモータへ値設定
        motor_r_val = MOTOR_STOP
        motor_l_val = MOTOR_STOP
        if self.move_status == MOVE_INPROGRESS:
            motor_r_val = MOTOR_AHEAD + bias
            motor_l_val = MOTOR_AHEAD - bias
        self.motor_r.changeDuty(motor_r_val)
        self.motor_l.changeDuty(motor_l_val)

#--------------------
    def main(self):
        # 状態遷移
        self.change_move_status()
        # 状態返却
        self.return_move_status()
        # 動作管理
        self.move()


def foot_main_py():
    # 初期化宣言 : このソフトウェアは"mileage_py_node"という名前
    rospy.init_node('foot_main_py_node', anonymous=True)
    # インスタンスの作成 
    foot = FootMain()
    # 処理周期の設定
    r = rospy.Rate(CYCLES)

    print("[foot_main] start")
    # ctl +　Cで終了しない限りwhileループで処理し続ける
    while not rospy.is_shutdown():
        # メイン処理
        foot.main()
        #
        r.sleep()

if __name__ == '__main__':
    try:
        foot_main_py()

    except rospy.ROSInterruptException:
        print("[foot_main] end")
