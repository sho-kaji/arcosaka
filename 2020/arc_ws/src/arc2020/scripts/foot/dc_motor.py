 # -*- coding: utf-8 -*-
import pigpio
import time

from PIGPIO_SWITCH import __pigpio__

PWM_PIN = [25,24] #PWM IN1,IN2
FREQ = 100
RANGE = 255
DUTY_MAX = 50*0.01

class DCMotor(object):

    def __init__(self, pin0, pin1):
        self.PINS = [pin0,pin1]
        self.pi = pigpio.pi()
        self.duty = 0
        print("PIN " + str(self.PINS[0]) + ":" + str(self.PINS[1]))
        if __pigpio__:
            for j in range(2):
                self.pi.set_mode(self.PINS[j], pigpio.OUTPUT)  #pigpioで制御するピンの指定
                self.pi.set_PWM_frequency(self.PINS[j], FREQ)  #PWMの周波数(Hz)を指定。デフォルトは800。
                self.pi.set_PWM_range(self.PINS[j], RANGE)  #PWM値(μs)の最大値を指定。指定可能な値は25〜40000。デフォルトは255。
   
    def  __del__(self):
       self.pi.stop()

    def changeDuty(self,inDuty):
        self.duty = inDuty * DUTY_MAX 
        if __pigpio__:
            if inDuty > 0:  #正転
                self.pi.set_PWM_dutycycle(self.PINS[0], self.duty) #デューティー比変更
                self.pi.set_PWM_dutycycle(self.PINS[1], 0)
            elif inDuty < 0:  #逆転
                self.duty = (-1)*self.duty
                self.pi.set_PWM_dutycycle(self.PINS[0], 0)
                self.pi.set_PWM_dutycycle(self.PINS[1], self.duty) #デューティー比変更
            else:
                # 停止はどうかくのか考え中
                self.pi.set_PWM_dutycycle(self.PINS[0], 0)
                self.pi.set_PWM_dutycycle(self.PINS[1], 0) #デューティー比変更
        # print("duty "+str(inDuty)+" "+str(self.duty))
        time.sleep(0.5)



