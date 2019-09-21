#!/usr/bin/python
# -*- coding: utf-8 -*-
# pylint: disable=E1101
"""
モーターテスト
"""

import mortor

def main():
    mc = mortor.MortorClass()
    while True:
        try:
            val = input('number:')
            int_tmp = int(val)
            if int_tmp < 0:
                break
            mc.move_servo(0,int_tmp)
        except KeyboardInterrupt:
            print("Ctrl+Cで停止しました")
            break
        except :
            pass


if __name__ == '__main__':
    main()
