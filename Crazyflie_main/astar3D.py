from __future__ import annotations
from typing import Iterator, Tuple, TypeVar, Optional
import heapq 
import math

T = TypeVar('T')
Location = TypeVar('Location')
GridLocation = Tuple[int, int, int]

# Funcion de cola de mensajes
class PriorityQueue:
    def __init__(self):
        self.elements: list[tuple[float, T]] = []
    
    def empty(self) -> bool:
        return not self.elements
    
    def put(self, item: T, priority: float):
        heapq.heappush(self.elements, (priority, item))
    
    def get(self) -> T:
        return heapq.heappop(self.elements)[1]  

# Función de grids
class GridWithWeights:
    def __init__(self, width: int, height: int, depth: int):
        self.width = width
        self.height = height
        self.depth = depth
        self.walls: list[GridLocation] = []
        self.weights: dict[GridLocation, float] = {}
    
    def in_bounds(self, id: GridLocation) -> bool:
        (x, y, z) = id
        return 0 <= x < self.width and 0 <= y < self.height and 0 <= z < self.depth
    
    def passable(self, id: GridLocation) -> bool:
        return id not in self.walls
    
    def neighbors(self, id: GridLocation) -> Iterator[GridLocation]:
        (x, y, z) = id
        neighbors = [(x + dx, y + dy, z + dz) for dx in range(-1, 2) for dy in range(-1, 2) for dz in range(-1, 2) if dx != 0 or dy != 0 or dz != 0]
        # see "Ugly paths" section for an explanation:
        if (x + y) % 2 == 0: neighbors.reverse()  # Cambiado a 3D
        results = filter(self.in_bounds, neighbors)
        results = filter(self.passable, results)
        return results
    
    def cost(self, from_node: GridLocation, to_node: GridLocation) -> float:      
        prev_cost = self.weights.get(to_node, 1) 
        nudge = 0
        (x1, y1, z1) = from_node
        (x2, y2, z2) = to_node
        if (x1 + y1) % 2 == 0 and x2 != x1: nudge = 1
        if (x1 + y1) % 2 == 1 and y2 != y1: nudge = 1
        return prev_cost + 0.001 * nudge

# Algoritmo Dijkstra
def dijkstra_search(graph: GridWithWeights, start: Location, goal: Location):
    frontier = PriorityQueue()
    frontier.put(start, 0)
    came_from: dict[Location, Optional[Location]] = {}
    cost_so_far: dict[Location, float] = {}
    came_from[start] = None
    cost_so_far[start] = 0
    
    while not frontier.empty():
        current: Location = frontier.get()
        
        if current == goal:
            break
        
        for next in graph.neighbors(current):
            new_cost = cost_so_far[current] + graph.cost(current, next)
            if next not in cost_so_far or new_cost < cost_so_far[next]:
                cost_so_far[next] = new_cost
                priority = new_cost
                frontier.put(next, priority)
                came_from[next] = current
    
    return came_from, cost_so_far


# Actualiza la función heuristic para 3D
def heuristic(a: GridLocation, b: GridLocation) -> float:
    (x1, y1, z1) = a
    (x2, y2, z2) = b
    return abs(x1 - x2) + abs(y1 - y2) + abs(z1 - z2)

# Algoritmo A*
def a_star_search(graph: GridWithWeights, start: Location, goal: Location):
    frontier = PriorityQueue()
    frontier.put(start, 0)
    came_from: dict[Location, Optional[Location]] = {}
    cost_so_far: dict[Location, float] = {}
    came_from[start] = None
    cost_so_far[start] = 0
    
    while not frontier.empty():
        current: Location = frontier.get()
        
        if current == goal:
            break
        
        for next in graph.neighbors(current):
            new_cost = cost_so_far[current] + graph.cost(current, next)
            if next not in cost_so_far or new_cost < cost_so_far[next]:
                cost_so_far[next] = new_cost
                priority = new_cost + heuristic(next, goal)
                frontier.put(next, priority)
                came_from[next] = current
    
    return came_from, cost_so_far


# Algoritmo que reconstruye el path
def reconstruct_path(came_from: dict[Location, Location],
                     start: Location, goal: Location) -> list[Location]:

    current: Location = goal
    path: list[Location] = []
    if goal not in came_from: # no path was found
        return []
    while current != start:
        path.append(current)
        current = came_from[current]
    path.append(start) # optional
    path.reverse() # optional
    return path


# Funciones para pintar graficas
def draw_tile(graph, id, style):
    r = " . "
    if 'number' in style and id in style['number']: r = " %-2d" % style['number'][id]
    if 'point_to' in style and style['point_to'].get(id, None) is not None:
        (x1, y1, z1) = id
        (x2, y2, z1) = style['point_to'][id]
        if x2 == x1 + 1: r = " > "
        if x2 == x1 - 1: r = " < "
        if y2 == y1 + 1: r = " v "
        if y2 == y1 - 1: r = " ^ "
    if 'path' in style and id in style['path']:   r = " @ "
    if 'start' in style and id == style['start']: r = " A "
    if 'goal' in style and id == style['goal']:   r = " Z "
    if id in graph.walls: r = "###"
    return r

def draw_grid(graph, **style):
    for z in range(graph.depth):
        print("___" * graph.width)
        for y in range(graph.height):
            for x in range(graph.width):
                print("%s" % draw_tile(graph, (x, y, z), style), end="")
            print()
        print("~~~" * graph.width)


def obtener_vecinos(dimensiones_maximas, lista_puntos):
    vecinos = []

    for punto in lista_puntos:
        x, y, z = punto
     
        for dx in [-1, 0, 1]:
                for dy in [-1, 0, 1]:
                    for dz in [-1, 0, 1]:
                        vecino = (x + dx, y + dy, z)
                        
                        # Verifica si el vecino está dentro de las dimensiones máximas
                        dentro_limites = all(0 <= coord < dim_max for coord, dim_max in zip(vecino, dimensiones_maximas))

                        # Verifica si el vecino no está en la lista original
                        no_en_lista_original = vecino not in lista_puntos

                        if dentro_limites and no_en_lista_original:
                            vecinos.append(vecino)

    return vecinos

# Funcion para añadir angulos en path
def agregar_angulo_entre_puntos(path):
    path_con_angulos = []

    path_con_angulos.append(path[0] + (0,))

    for i in range(1, len(path)):
        punto_actual = path[i]
        punto_anterior = path[i - 1]
        
        # Calcular ángulo en radianes utilizando atan2
        angulo_radianes = math.atan2((punto_actual[1] - punto_anterior[1]), (punto_actual[0] - punto_anterior[0]))
        
        # Agregar el ángulo al punto actual
        punto_con_angulo = punto_actual + (angulo_radianes,)
        
        # Agregar el punto con ángulo a la lista
        path_con_angulos.append(punto_con_angulo)
    
    return path_con_angulos

# Funciones de prueba de interpolado
def interpolacion_lineal_entre_puntos(punto1, punto2, cantidad_puntos):
    puntos_interpolados = []
    
    for i in range(1, cantidad_puntos + 1):
        factor_interpolacion = i / (cantidad_puntos + 1)
        x_interpolado = punto1[0] + factor_interpolacion * (punto2[0] - punto1[0])
        y_interpolado = punto1[1] + factor_interpolacion * (punto2[1] - punto1[1])
        z_interpolado = punto1[2] + factor_interpolacion * (punto2[2] - punto1[2])
        
        puntos_interpolados.append((x_interpolado, y_interpolado, z_interpolado))#, punto2[3]))
    
    return puntos_interpolados

def interpolar_camino(path):
    path_interpolado = [path[0]]
    
    for i in range(1, len(path)):
        punto_anterior = path[i - 1]
        punto_actual = path[i]
        
        # Interpolar entre puntos anteriores y actuales
        puntos_interpolados = interpolacion_lineal_entre_puntos(punto_anterior, punto_actual, cantidad_puntos=1)
        
        # Agregar puntos interpolados al nuevo camino
        path_interpolado.extend(puntos_interpolados)
        path_interpolado.append(punto_actual)
    
    return path_interpolado
