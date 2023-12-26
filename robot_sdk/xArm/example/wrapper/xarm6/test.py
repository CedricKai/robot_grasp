#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2019, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

"""
Description: Move Joint
"""

import os
import sys
import time
import math

sys.path.append(os.path.join(os.path.dirname(__file__), '../../..'))

from xarm.wrapper import XArmAPI


#######################################################
"""
Just for test example
"""
if len(sys.argv) >= 2:
    ip = sys.argv[1]
else:
    try:
        from configparser import ConfigParser
        parser = ConfigParser()
        parser.read('../robot.conf')
        ip = parser.get('xArm', 'ip')
    except:
        ip = input('Please input the xArm ip address:')
        if not ip:
            print('input error, exit')
            sys.exit(1)
########################################################


arm = XArmAPI(ip)
arm.motion_enable(enable=True)
arm.set_mode(0)
arm.set_state(state=0)



rand_pos = [[438, 0, 168, -178, 20, -3]]

aa =[[350, 0, 150, 180, 0, 0], [355, -60, 155, -170, 0, 0], [355, 70, 155, 170, 0, 0],
                         [345, 55, 155, 168, 0, 10], [350, 85, 155, 168, 0, -15], [345, 55, 138, 172, 0, 0],
                         [335, 5, 170, -178, 0, -15], [330, -55, 140, -165, 2, -16], [390, -65, 140, -167, 5, 10],
                         [420, -65, 150, -166, 6, 25], [320, 135, 175, 162, -12, -22], [423, -25, 197, -178, 10, 24],
                         [403, 30, 176, 176, 14, -24], [463, 0, 195, -175, 16, 30], [480, -18, 179, -171, 22, 20],
                         [387, -40, 162, -171, 11, -5], [310, -60, 152, -171, -10, -5], [356, -68, 148, -166, 3, -5],
                         [400, 20, 177, 175, 13, 5], [438, 0, 168, -178, 20, -3]]


for pos in rand_pos:
    arm.set_position(*pos, speed=50, mvacc=50, wait=True)


arm.disconnect()
