

import math
from enum import Enum


def DeteccionObstaculo(front_value, left_value, back_value, right_value):
        
        limite = 0.5   ### Debe ser un valor en el intervalo (0.3, 2) [m]
        flag_front = True
        flag_left = True
        flag_back = True
        flag_right = True
        
        if front_value > limite: 
            flag_front = True
        else:                    
            flag_front = False
        
        if left_value > limite:  
            flag_left = True
        else:                    
            flag_left = False
        
        if back_value > limite:    
            flag_back = True
        else:                       
            flag_back = False
        
        if right_value > limite:    
            flag_right = True
        else:                       
            flag_right = False
        


        return flag_front, flag_left, flag_back, flag_right
        
       










