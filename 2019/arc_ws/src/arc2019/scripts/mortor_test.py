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
            port = input('  port:')
            int_port = int(port)
            if int_port < 0:
                break

            val = input('number:')
            int_tmp = int(val)
            if int_tmp < 0:
                break

            mc.move_servo(0, int_tmp,False)
        except KeyboardInterrupt:
            print("Ctrl+Cで停止しました")
            break
        except :
            pass
    mc.endfnc()


if __name__ == '__main__':
    main()
