#import numpy as np
#from astar3D import *

# Cargar la matriz desde el archivo .txt
#obstaculos = np.loadtxt('casa_matriz_5.txt', delimiter='\t')
#obstaculos_filtrados = [(int(x), int(y), int(z)) for x, y, z in obstaculos]

# Creo el grid(mapa) con las paredes siendo las de la matriz del mapa
#g = GridWithWeights(21, 21, 9)
#g.walls = obstaculos_filtrados

#start, goal = (1, 1, 1), (3, 10, 6)
#came_from, cost_so_far = a_star_search(g, start, goal)
#draw_grid(g, path=reconstruct_path(came_from, start=start, goal=goal))
#print(reconstruct_path(came_from, start=start, goal=goal))




import numpy as np
from astar3D import *

# Cargar la matriz desde el archivo .txt
obstaculos = np.loadtxt('casa_matriz.txt', delimiter='\t')
obstaculos_filtrados = [(int(x), int(y), int(z)) for x, y, z in obstaculos]
obstaculos_seguridad = obtener_vecinos([21,21,9],obstaculos_filtrados)

# Creo el grid(mapa) con las paredes siendo las de la matriz del mapa
g = GridWithWeights(21, 21, 9)
g.walls = obstaculos_filtrados
g.weights = {loc: 5 for loc in obstaculos_seguridad}

start, goal = (2, 19, 1), (3, 10, 6)
came_from, cost_so_far = a_star_search(g, start, goal)
draw_grid(g, path=reconstruct_path(came_from, start=start, goal=goal))
path=reconstruct_path(came_from, start=start, goal=goal)
print(path)
print(path[1])

# Prueba de aÃ±adir angulo a las rutas
path_ang = agregar_angulo_entre_puntos(path)
print(path_ang)

# Prueba de interpolado
path_interpolado = interpolar_camino(path_ang)
print(path_interpolado)
