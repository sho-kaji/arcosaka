#!/usr/bin/env python
# -*- coding: utf-8 -*-
# pylint: disable=E1101,C0325

import time
from pcf8574 import PCF8574
i2c_port_num = 1
pcf_address = 0x20
pcf = PCF8574(i2c_port_num, pcf_address)

while True:
    print(f'{pcf.port}')
    print('')
    time.sleep(0.1)
