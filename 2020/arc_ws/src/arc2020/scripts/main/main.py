#!/usr/bin/python
# -*- coding: utf-8 -*-
# license removed for brevity

# pythonでROSのソフトウェアを記述するときにimportするモジュール
import rospy

# gpio制御用
import pigpio
import time

# 自分で定義したmessageファイルから生成されたモジュール
#受信
from arc2020.msg import webapp
from arc2020.msg import arm
from arc2020.msg import foot
from arc2020.msg import database
from arc2020.msg import emgstp
from arc2020.msg import volcurmeas

#送信
from arc2020.msg import main


# 定数などの定義ファイルimport


CYCLES = 60 

class Main(object):
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
        # webapp⇒main
        self.sub_webapp  = rospy.Subscriber('webapp', webapp, self.webappCallback, queue_size=1)
        
        # arm⇒main
        self.sub_arm  = rospy.Subscriber('arm', arm, self.armCallback, queue_size=1)
        
        # foot⇒main
        self.sub_foot  = rospy.Subscriber('foot', foot, self.footCallback, queue_size=1)
        
        # database⇒main
        self.sub_database  = rospy.Subscriber('database', database, self.databaseCallback, queue_size=1)

        # emgstp⇒main
        self.sub_emgstp  = rospy.Subscriber('emgstp', emgstp, self.emgstpCallback, queue_size=1)

        # volcurmeas⇒main
        self.sub_volcurmeas  = rospy.Subscriber('volcurmeas', volcurmeas, self.volcurmeasCallback, queue_size=1)


        # 送信作成
        # main⇒各モジュール
        self.pub_main = rospy.Publisher('main', main, queue_size=100)
        self.msg_main = main()

        # 送信メッセージ・受信バッファ初期化
        self.clearMsg()
        
#--------------------
# 送信メッセージ・受信バッファ初期化
    def clearMsg(self):
        """
        publishするメッセージのクリア
        """
        # main⇒foot
        self.msg_main.foot_operation = 0
        for i in range(30):
            self.msg_main.foot_x_cordinate.append(0)
        self.msg_main.foot_mode = 0
        self.msg_main.foot_motor_l = 0
        self.msg_main.foot_motor_r = 0
        
        # main⇒arm
        self.msg_main.arm_retorg_req = 0
        self.msg_main.arm_drill_req = 0
        for i in range(30):
            self.msg_main.arm_drill_width.append(0)
        self.msg_main.arm_ridge_width = 0
        self.msg_main.arm_ridge_height = 0
        
        # main⇒database
        self.msg_main.database_inforeq = 0
        self.msg_main.database_vegetable_num1 = 0
        self.msg_main.database_vegetable_num2 = 0
        self.msg_main.database_ridge_width = 0
        self.msg_main.database_ridge_length = 0
        
        # main⇒webapp
        self.msg_main.webapp_loginpermit = 0
        for i in range(30):
            self.msg_main.webapp_mainseedlist.append(0)
        for i in range(30):
            self.msg_main.webapp_subseedlist.append(0)
        self.msg_main.webapp_batterycharge = 0
        self.msg_main.webapp_blueprint = 0
        self.msg_main.webapp_mileage = 0
        self.msg_main.webapp_completionrate = 0
        self.msg_main.webapp_operatingstate = 0
        
        
        # webapp⇒main
        self.username = 0
        self.password = 0
        self.inforeq = 0
        self.mainseed = 0
        self.subseed = 0
        self.ridge_length = 0
        self.ridge_width = 0
        self.ridge_height = 0
        self.blueprintreq = 0
        self.movestartreq = 0
        self.movestopreq = 0
        self.continueselect = 0
        
        # arm⇒main
        self.drillstatus = 0
        self.retorgstatus = 0
        
        # foot⇒main
        self.movestatus = 0
        
        # database⇒main
        for i in range(30):
            self.mainseedlist.append(0)
        for i in range(30):
            self.subseedlist.append(0)
        self.blueprint = 0
        for i in range(30):
            self.coordinates_x.append(0)
        for i in range(30):
            self.coordinates_y.append(0)
        
        # emgstp⇒main
        self.emgstopstate = 0
        
        # volcurmeas⇒main
        self.batterycharge = 0
        


#--------------------
# 受信コールバック
    def webappCallback(self, webapp_msg):
        """
        クライアントの受信コールバック
        """
        self.username = webapp_msg.username
        self.password = webapp_msg.password
        self.inforeq = webapp_msg.inforeq
        self.mainseed = webapp_msg.mainseed
        self.subseed = webapp_msg.subseed
        self.ridge_length = webapp_msg.ridge_length
        self.ridge_width = webapp_msg.ridge_width
        self.ridge_height = webapp_msg.ridge_height
        self.blueprintreq = webapp_msg.blueprintreq
        self.movestartreq = webapp_msg.movestartreq
        self.movestopreq = webapp_msg.movestopreq
        self.continueselect = webapp_msg.continueselect

#--------------------
# 受信コールバック
    def armCallback(self, arm_msg):
        """
        クライアントの受信コールバック
        """
        self.drillstatus = arm_msg.drillstatus
        self.retorgstatus = arm_msg.retorgstatus

#--------------------
# 受信コールバック
    def footCallback(self, foot_msg):
        """
        クライアントの受信コールバック
        """
        self.movestatus = foot_msg.movestatus

#--------------------
# 受信コールバック
    def databaseCallback(self, database_msg):
        """
        クライアントの受信コールバック
        """
        for i in range(30):
            self.mainseedlist[i] = database_msg.mainseedlist[i]
        for i in range(30):
            self.subseedlist[i] = database_msg.subseedlist[i]
        self.blueprint = database_msg.blueprint
        for i in range(30):
            self.coordinates_x[i] = database_msg.coordinates_x[i]
        for i in range(30):
            self.coordinates_y[i] = database_msg.coordinates_y[i]

#--------------------
# 受信コールバック
    def emgstpCallback(self, emgstp_msg):
        """
        クライアントの受信コールバック
        """
        self.emgstopstate = emgstp_msg.emgstopstate

#--------------------
# 受信コールバック
    def volcurmeasCallback(self, volcurmeas_msg):
        """
        クライアントの受信コールバック
        """
        self.batterycharge = volcurmeas_msg.batterycharge

#--------------------
# メイン関数
    def main(self):
        
        # メッセージを発行する
        self.pub_main.publish(self.msg_main)
        
#--------------------
def main_py():
    # 初期化宣言 : このソフトウェアは"maindebug_py_node"という名前
    rospy.init_node('main_py_node', anonymous=True)
    # インスタンスの作成 
    main = Main()
    # 処理周期の設定
    r = rospy.Rate(CYCLES)

    print("[main] start")
    # ctl +　Cで終了しない限りwhileループで処理し続ける
    while not rospy.is_shutdown():
        # メイン処理
        main.main()
        #
        r.sleep()

if __name__ == '__main__':
    try:
        maindebug_py()

    except rospy.ROSInterruptException:
        print("[main] end")
