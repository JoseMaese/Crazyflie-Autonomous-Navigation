"""
Incluye:
    - Parada antes de chocar con obstaculos en todas direcciones
    - Ruta predefinida (circular)
    - Reconocimiento de figuras con redes neuronales (modelo YOLO)
    
"""
from controller import Robot
from controller import Keyboard
from controller import Supervisor

import math
from math import cos, sin, floor, atan2

from pid_controller import pid_velocity_fixed_height_controller
from ruta_predefinida import RutaCircular
from evita_obstaculos import DeteccionObstaculo

import cv2
import numpy as np
from io import BytesIO
from PIL import Image
from YOLO_model_real_time import load_image, post_process
from pid_vel import QuadrotorController
import matplotlib.pyplot as plt
from astar3D import *


import threading
import csv

FLYING_ATTITUDE = 2

if __name__ == '__main__':
    
    def exportar_a_csv(filename, data):
        with open(filename, 'w', newline='') as csvfile:
            fieldnames = ['Tiempo', 'Posición X', 'Posición Y', 'Posición Z', 'Objetivo X', 'Objetivo Y', 'Objetivo Z']
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
        
            writer.writeheader()
            for row in data:
                writer.writerow({
                    'Tiempo': row[0],
                    'Posición X': row[1],
                    'Posición Y': row[2],
                    'Posición Z': row[3],
                    'Objetivo X': row[4],
                    'Objetivo Y': row[5],
                    'Objetivo Z': row[6],
                })
                
    def exportar_path_a_csv(filename, data):
        with open(filename, 'w', newline='') as csvfile:
            fieldnames = ['Path X', 'Path Y', 'Path Z']
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
        
            writer.writeheader()
            for row in data:
                writer.writerow({
                    'Path X': row[0],
                    'Path Y': row[1],
                    'Path Z': row[2],
                })            
    historial = []
            
                
    robot = Robot()
    timestep = int(robot.getBasicTimeStep())

    # Initialize motors
    m1_motor = robot.getDevice("m1_motor")
    m1_motor.setPosition(float('inf'))
    m1_motor.setVelocity(-1)
    m2_motor = robot.getDevice("m2_motor")
    m2_motor.setPosition(float('inf'))
    m2_motor.setVelocity(1)
    m3_motor = robot.getDevice("m3_motor")
    m3_motor.setPosition(float('inf'))
    m3_motor.setVelocity(-1)
    m4_motor = robot.getDevice("m4_motor")
    m4_motor.setPosition(float('inf'))
    m4_motor.setVelocity(1)

    # Initialize Sensors
    imu = robot.getDevice("inertial_unit")
    imu.enable(timestep)
    gps = robot.getDevice("gps")
    gps.enable(timestep)
    gyro = robot.getDevice("gyro")
    gyro.enable(timestep)
    camera = robot.getDevice("camera")
    camera.enable(timestep)
    range_front = robot.getDevice("range_front")
    range_front.enable(timestep)
    range_left = robot.getDevice("range_left")
    range_left.enable(timestep)
    range_back = robot.getDevice("range_back")
    range_back.enable(timestep)
    range_right = robot.getDevice("range_right")
    range_right.enable(timestep)

    # Get keyboard
    keyboard = Keyboard()
    keyboard.enable(timestep)

    # Initialize variables

    past_x_global = 0
    past_y_global = 0
    past_time = 0
    first_time = True
    IniciarCamara = True
    IniciarGraficas = True
    crearRecorrido = True
    personaDetectada = 0
    #t_espera_persona = 0
    #t_detectada = 0
    total_simulation_time = 0

    # Crazyflie velocity PID controller
    PID_crazyflie = pid_velocity_fixed_height_controller()
    PID_position = QuadrotorController()
    PID_update_last_time = robot.getTime()
    sensor_read_last_time = robot.getTime()
    
    # Set initial and target position
    current_position = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    target_position = np.array([3, 10, 6, 0.0, 0.0, 0.0]) 
    height_desired = FLYING_ATTITUDE
    
    # Etiquetas posibles de la red neuronal
    labels = open('coco.names').read().strip().split('\n')
    captura = cv2.VideoCapture(0)
    
    # Cargamos matriz A*
    obstaculos = np.loadtxt('casa_matriz_5.txt', delimiter='\t')
    obstaculos_filtrados = [(int(x), int(y), int(z)) for x, y, z in obstaculos]
    
    # Creo el grid(mapa) con las paredes siendo las de la matriz del mapa
    g = GridWithWeights(21, 21, 9)
    g.walls = obstaculos_filtrados
    obstaculos_seguridad = obtener_vecinos([21,21,9],obstaculos_filtrados)
    g.weights = {loc: 5 for loc in obstaculos_seguridad}
    
    autonomous_mode = False

    print("\n")

    print("====== Controls =======\n\n")
    
    print(" The Crazyflie can be controlled from your keyboard!\n")
    print(" All controllable movement is in body coordinates\n")
    print("- Use the up, back, right and left button to move in the horizontal plane\n")
    print("- Use Q and E to rotate around yaw\n ")
    print("- Use W and S to go up and down\n ")
    print("- Press A to start autonomous mode\n")
    print("- Press D to disable autonomous mode\n")

    ###########################################################################
    #### VISION ARTIFICIAL: uso de red YOLO para reconocimiento de figuras ####
    ###########################################################################
    def VisionArtificial():
        global personaDetectada
        global t_detectada
        CamaraEncendida = True
        while CamaraEncendida:
            video_reader = camera.getImage()
            #width, height = camera.getWidth(), camera.getHeight()

            img_np = np.frombuffer(video_reader, dtype=np.uint8).reshape((324, 324, 4))
            img_np = img_np[:,:,:3]
        
            boxes = load_image(img_np)
            (image, personaDetectada, t_detectada) = post_process(img_np, boxes, 0.5, total_simulation_time)
            
            #print(personaDetectada, t_detectada)
            cv2.imshow("Recognition", image)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('v'):
                cv2.destroyAllWindows()
                video_reader.release()
                CamaraEncendida = False
       
    Hilo_Camara = threading.Thread(target=VisionArtificial)

    ###########################################################################
    #### GRAFICAS: PID alto nivel #############################################
    ########################################################################### 

    # Main loop:
    while robot.step(timestep) != -1:     
        dt = robot.getTime() - past_time
        total_simulation_time += dt
        actual_state = {}

        if first_time:
            past_x_global = gps.getValues()[0]
            past_y_global = gps.getValues()[1]
            past_time = robot.getTime()
            first_time = False

        # Get sensor data
        roll = imu.getRollPitchYaw()[0]
        pitch = imu.getRollPitchYaw()[1]
        yaw = imu.getRollPitchYaw()[2]
        yaw_rate = gyro.getValues()[2]
        x_global = gps.getValues()[0]
        v_x_global = (x_global - past_x_global)/dt
        y_global = gps.getValues()[1]
        v_y_global = (y_global - past_y_global)/dt
        altitude = gps.getValues()[2]

        # Get body fixed velocities
        cos_yaw = cos(yaw)
        sin_yaw = sin(yaw)
        v_x = v_x_global * cos_yaw + v_y_global * sin_yaw
        v_y = - v_x_global * sin_yaw + v_y_global * cos_yaw

        # Initialize values
        desired_state = [0, 0, 0, 0]
        forward_desired = 0
        sideways_desired = 0
        yaw_desired = 0
        height_diff_desired = 0
        
        # get range in meters
        range_front_value = range_front.getValue() / 1000
        range_right_value = range_right.getValue() / 1000
        range_left_value = range_left.getValue() / 1000
        range_back_value = range_left.getValue() / 1000
        
        flag_front, flag_left, flag_back, flag_right = DeteccionObstaculo(range_front_value, range_left_value, range_back_value, range_right_value)
        
        key = keyboard.getKey()
        while key > 0:
            if key == Keyboard.UP and flag_front:
                forward_desired += 0.5
            elif key == Keyboard.DOWN and flag_back:
                forward_desired -= 0.5
            elif key == Keyboard.RIGHT and flag_right:
                sideways_desired -= 0.5
            elif key == Keyboard.LEFT and flag_left:
                sideways_desired += 0.5
            elif key == ord('Q'):
                yaw_desired = + 1
            elif key == ord('E'):
                yaw_desired = - 1
            elif key == ord('W'):
                height_diff_desired = 0.1
            elif key == ord('S'):
                height_diff_desired = - 0.1
            elif key == ord('A'):
                if autonomous_mode is False:
                    autonomous_mode = True
                    crearRecorrido = True
                    print("Autonomous mode: ON")
            elif key == ord('D'):
                if autonomous_mode is True:
                    target_position = np.array([3, 10, 6, 0.0, 0.0, 0.0])
                    autonomous_mode = False
                    crearRecorrido = True
                    print("Autonomous mode: OFF")
            elif key == ord('1'):
                    target_position = np.array([3, 10, 6, 0.0, 0.0, 0.0])
                    crearRecorrido = True
                    print("Next target: MESA_A")
            elif key == ord('2'):
                    target_position = np.array([7, 15, 3, 0.0, 0.0, 0.0])
                    crearRecorrido = True
                    print("Next target: SILLON")
            elif key == ord('3'):
                    target_position = np.array([13, 5, 6, 0.0, 0.0, 0.0])
                    crearRecorrido = True
                    print("Next target: MESA_B")
            elif key == ord('I'):
                    exportar_a_csv('datos.csv', historial)
                    exportar_path_a_csv('datos_path.csv', path)
                    print("Graficas")
            key = keyboard.getKey()    
                 
                 
        if IniciarCamara:
            Hilo_Camara.start()
            IniciarCamara = False

        current_position = gps.getValues() + imu.getRollPitchYaw()

        ###############################################################
        #### MODO AUTONOMO ############################################ 
        ###############################################################
        
        if autonomous_mode:            
            if crearRecorrido:
                start = (floor(current_position[0]), floor(current_position[1]), floor(current_position[2]))
                goal = (target_position[0], target_position[1], target_position[2])
                came_from, cost_so_far = a_star_search(g, start, goal)
                path = reconstruct_path(came_from, start=start, goal=goal)
                path = interpolar_camino(path)
                print(len(path), 'destinos: ', path)
                draw_grid(g, path=reconstruct_path(came_from, start=start, goal=goal))
                pto_intermedio = 1
                buscando_direccion = True
                crearRecorrido = False
                print(g)
                               
                
            target_position[0:3] = path[pto_intermedio]
            target_position[0:3] = target_position[0:3] + 0.5
            
            alpha = atan2((target_position[1] - current_position[1]), (target_position[0]) - current_position[0])
            #print('Current: ', round(current_position[5], 3), '      Alpha: ', round(alpha, 3))
            
            if buscando_direccion: 
                if current_position[5] >= math.pi/2 and current_position[5]*alpha < 0:
                    current_position_desplazada = current_position[5] - math.pi
                    alpha_desplazada = alpha - math.pi/2
                    if current_position_desplazada > alpha_desplazada + 0.1:
                        yaw_desired = + 0.15
                    elif current_position_desplazada < alpha_desplazada - 0.1:
                        yaw_desired = - 0.15
                elif current_position[5] <= -math.pi/2 and current_position[5]*alpha < 0:
                    current_position_desplazada = current_position[5] - math.pi
                    alpha_desplazada = alpha - math.pi/2
                    if current_position_desplazada > alpha_desplazada + 0.1:
                        yaw_desired = + 0.15
                    elif current_position_desplazada < alpha_desplazada - 0.1:
                        yaw_desired = - 0.15       
                else:
                    if current_position[5] > alpha + 0.1:
                        yaw_desired = - 0.15
                    elif current_position[5] < alpha - 0.1:
                        yaw_desired = + 0.15
                    else:
                        buscando_direccion = False
            else:
                buscando_direccion = False
                if personaDetectada:                    
                    t_espera_persona = total_simulation_time - t_detectada
                    cmd_vel_x = 0 
                    cmd_vel_y = 0
                    cmd_ang_w = 0                         
                    height_diff_desired = 0
                    print('Persona detectada')
                else:  
                    cmd_vel_x, cmd_vel_y, cmd_ang_w = PID_position.control_quadrotor(current_position, target_position, dt)                          
                    if target_position[2] > current_position[2]:
                        height_diff_desired = min(target_position[2] - current_position[2], 0.1) 
                    if target_position[2] < current_position[2]:
                        height_diff_desired = max(target_position[2] - current_position[2], -0.1)
                #print('Destino ', pto_intermedio, ': ', target_position)
                     
                sideways_desired = cmd_vel_y
                forward_desired = cmd_vel_x
                #print('X = ', round(forward_desired, 2), '     Y = ', round(sideways_desired, 2))
                distTargetX = abs(target_position[0] - current_position[0])
                distTargetY = abs(target_position[1] - current_position[1])
                distTargetZ = abs(target_position[2] - current_position[2])
                #print('X = ', round(distTargetX, 2), '     Y = ', round(distTargetY, 2),     '     Z = ', round(distTargetZ, 2))
                if  distTargetX < 0.2 and distTargetY < 0.2 and distTargetZ < 0.3:
                    buscando_direccion = True
                    pto_intermedio = pto_intermedio + 1
                    if pto_intermedio == len(path):
                        target_position = np.array([3, 10, 6, 0.0, 0.0, 0.0])
                        crearRecorrido = True
                        autonomous_mode = False
                        
        
        # PID velocity controller with fixed height
        height_desired += height_diff_desired * dt
        motor_power = PID_crazyflie.pid(dt, forward_desired, sideways_desired,
                                        yaw_desired, height_desired,
                                        roll, pitch, yaw_rate,
                                        altitude, v_x, v_y)
                                        
        ##############################################################################
        historial.append([total_simulation_time,current_position[0], current_position[1], current_position[2],target_position[0], target_position[1], target_position[2]])
        ##############################################################################
        
        m1_motor.setVelocity(-motor_power[0])
        m2_motor.setVelocity(motor_power[1])
        m3_motor.setVelocity(-motor_power[2])
        m4_motor.setVelocity(motor_power[3])

        past_time = robot.getTime()
        past_x_global = x_global
        past_y_global = y_global


cv2.destroyAllWindows()
video_reader.release()






