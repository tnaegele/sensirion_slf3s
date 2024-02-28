#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Example for sensirion_slf3s package on https://github.com/tnaegele/sensirion_slf3s
requires at least Python 3.10

@author: Tobias E. Naegele, 02/2024

Copyright (c) 2024 Tobias E. Naegele
"""

from sensirion_slf3s import slf3s,find_sensor_port
from sensirion_shdlc_driver import ShdlcSerialPort, ShdlcConnection
import time


sensor_port = find_sensor_port()

with (ShdlcSerialPort(port=sensor_port, baudrate=115200) as port, 
        slf3s(ShdlcConnection(port), slave_address=0) as device):

    device.start(water=True)
    
    while True:
        flow,temp,flag = device.get_last()
        print(f'flow rate: {flow} ul/min, temperature: {temp} C, flag: {flag}')
        time.sleep(0.1)