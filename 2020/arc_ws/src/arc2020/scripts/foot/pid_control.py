#!/usr/bin/python
# -*- coding: utf-8 -*-
# license removed for brevity
# reference : https://monoist.atmarkit.co.jp/mn/articles/1007/26/news083.html

KP = 0
KI = 0
KD = 0

DELTA_TIME = 0.004

LINE_WEIGHT = 1
LINE_PARAM = [-3,-2,-1,0,0,1,2,3]
LINE_BIAS = 5

RANGE_BIAS = 5
RANGE_TARGET = 4.0

class PIDControl(object):
    """
    スイッチの動作を制御するクラス
    """
#--------------------
# コンストラクタ
    def __init__(self,delta_time = DELTA_TIME):
        """
        コンストラクタ
        """
        self.delta_time = delta_time
        self.diff_line_trace = [0.0,0.0]
        self.integral_line_trace = 0.0

        self.diff_range = [0.0,0.0]
        self.integral_range = 0.0
    # end

    def get_bias_line_trace(self,line):
        output = 0
        input = 0
        target = 0
        for i in range(len(LINE_PARAM)):
            input = input + line[i]*LINE_PARAM[i]*LINE_WEIGHT
        #output = self.pid_sample(input,target,self.diff_line_trace,self.integral_line_trace)
        output = input * LINE_BIAS
        return output
    # end
    def get_bias_mileage(self,line):
        output = 0
        
        return output
    # end

    def get_bias_side_range(self,right,left):
        input = right
        output = 0
        target = RANGE_TARGET
        if left > right:
            input = -1*left
        #output = self.pid_sample(input,target,self.diff_range,self.integral_range)
        output = input * RANGE_BIAS
        return output
    # end

    
    def pid_sample(self,input,target,diff,integral):
        diff[0] = diff[1]
        diff[1] = input - target
        integral = integral + (diff[0]+ diff[1])/2 * self.delta_time

        p = KP * self.diff[1]
        i = KI * integral
        d = KD * (diff[1] - diff[1])/DELTA_TIME

        return p+i+d

    


        





#--------------------
    def control(self):
        return 
