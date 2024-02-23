# -*- coding: utf-8 -*-
#
#  ...........       ____  _ __
#  |  ,-^-,  |      / __ )(_) /_______________ _____  ___
#  | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
#  | / ,..´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#     +.......   /_____/_/\__/\___/_/   \__,_/ /___/\___/
#
#  GNU general public license v3.0
#
#  Copyright (C) 2023 Bitcraze AB
#

"""
file: wall_following.py

Class for the wall following demo

This is a python port of c-based app layer example from the Crazyflie-firmware
found here https://github.com/bitcraze/crazyflie-firmware/tree/master/examples/
demos/app_wall_following_demo

Author:   Kimberly McGuire (Bitcraze AB)
"""


import math
from enum import Enum


def RutaCircular():
        
        command_velocity_x_temp = 0.3
        command_velocity_y_temp = 0.0
        command_angle_rate_temp = 0.3


              
        command_velocity_x = command_velocity_x_temp
        command_velocity_y = command_velocity_y_temp
        command_yaw_rate = command_angle_rate_temp


        return command_velocity_x, command_velocity_y, command_yaw_rate









