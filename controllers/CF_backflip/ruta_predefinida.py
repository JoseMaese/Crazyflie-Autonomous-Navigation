
import math
from enum import Enum


def backflip(roll, pitch, yaw_rate, altitude):
        
    # Calcular las velocidades de comando para el backflip
    command_velocity_x = 0
    command_velocity_y = 0
    command_yaw_rate = 0
    pitch_desired = pitch +1
    
    print(pitch)
    
    return command_velocity_x, command_velocity_y, command_yaw_rate, pitch_desired









