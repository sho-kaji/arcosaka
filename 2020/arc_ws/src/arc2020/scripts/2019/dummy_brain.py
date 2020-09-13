#!/usr/bin/env python
# -*- coding: utf-8 -*-
# pylint: disable=E1101,C0103,C0325
"""
"""

import sys
import json
import rospy
import roslib

from arc2020.msg import client

from params import MODE, TARGET

class Server():
    """
    """
    def __init__(self):
        print("init")
        # initialize gpio

    def callback(self, msg):
        """
        メッセージを受信したときに呼び出し
        """
        #モード変更確認
        #print('hand.req', msg.hand_req)
        print('hand.mode', msg.mode)
        print('hand.target', msg.target)
        print("==============================")
     
    def on_connect(*args):
        rospy.loginfo('socket.io connected.')

    def on_disconnect(*args):
        rospy.loginfo('socket.io disconnected.')


def server_py():
    """
    cleintのテスト用serverのメイン
    """

    server = Server()
    rospy.init_node('dummy_brain_py_node', anonymous=True)
    #server.protocol.Subscriber('client', brain, server.callback, queue_size=1)
    rospy.Subscriber('client2', client, server.callback, queue_size=1)
    print("start")
    rospy.spin()

if __name__ == '__main__':
    server_py()
