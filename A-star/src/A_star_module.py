
import numpy as np
from math import sqrt
import heapq
import matplotlib.pyplot as plt

class Node:
    def __init__(self, position:tuple[int, int], g:float = float('inf'), h:float = 0.0, parent:dict = None):
        self.position = position
        self.g = g
        self.h = h
        self.f = g + h
        self.parent = parent
        

class Path:
    def __init__(self, grid:np.ndarray, start_pos:tuple[int, int], end_pos:tuple[int, int]):
        self.grid = grid
        self.start_posisiton = start_pos
        self.end_position = end_pos
        
    
    #calculate the heuristic value, use Euclidean distance
    #it can be modified to other kinds of distance 
    def calculate_heuristic(self, pos1:tuple[int, int], pos2:tuple[int, int])->float:
        x1, y1 = pos1
        x2, y2 = pos2
        return sqrt((x2 - x1)**2 + (y2 - y1)**2)
    
    #find possible movements from 8 directions of one cell
    #vacancy  -- 0
    #obstacle -- 1
    def get_node_neigbours(self, grid:np.ndarray, pos:tuple[int, int])->list[tuple[int,int]]:
        x, y = pos
        rows, cols = grid.shape
        
        movement = [
            (x+1, y), (x-1, y),
            (x, y+1), (x, y-1),
            (x+1, y+1), (x-1, y-1),
            (x+1, y-1), (x-1, y+1)
        ]
        
        result = []
        for nx, ny in movement:
            if grid[nx, ny] == 0 and 0 <= nx < rows and 0 <= ny < cols:
                # result.append(tuple(nx, ny))
                result.append((nx, ny))
        
        return result

    def get_path(self, goal_node:Node)->list[tuple[int,int]]:
        path = []
        current_node = goal_node
        
        while current_node is not None:
            path.append(current_node.position)
            current_node = current_node.parent
            
        return path[::-1] # from start to end
    
    #A star algorithm
    def find_path(self)->list[tuple[int,int]]:
        
        start_node = Node(self.start_posisiton, 0., self.calculate_heuristic(self.start_posisiton, self.end_position))
        
        open_list = [(start_node.f, self.start_posisiton)]
        open_dict = {self.start_posisiton: start_node}
        closed_set = set()
        
        while open_list:
            _, current_pos = heapq.heappop(open_list)
            current_node = open_dict[current_pos]
            
            if current_pos == self.end_position:
                return self.get_path(current_node)
            
            closed_set.add(current_pos)
            
            for neighbour_pos in self.get_node_neigbours(self.grid, current_pos):
                if neighbour_pos in closed_set:
                    continue
                
                g_temp = current_node.g + self.calculate_heuristic(current_pos, neighbour_pos)
                
                if neighbour_pos not in open_dict:
                    neighbour_node = Node(neighbour_pos,g_temp, self.calculate_heuristic(neighbour_pos, self.end_position), current_node)
                    heapq.heappush(open_list, (neighbour_node.f, neighbour_pos))
                    open_dict[neighbour_pos] = neighbour_node
                elif g_temp < open_dict[neighbour_pos].g:
                    neighbour_node = open_dict[neighbour_pos]
                    neighbour_node.g = g_temp
                    neighbour_node.f = g_temp + neighbour_node.h
                    neighbour_node.parent = current_node

        #not found
        return []
    
    
    #visualize
    def visualize_path(self, path:list[tuple[int, int]]):
        plt.figure(figsize=(10,10))
        plt.imshow(self.grid, cmap='binary')
        
        if path:
            path = np.array(path)
            plt.plot(path[:,1], path[:,0], 'b-', linewidth=3, label='Path')
            plt.plot(path[0,1], path[0,0], 'go', markersize=15, label='Start')
            plt.plot(path[-1,-1], path[-1,0], 'ro', markersize=15, label="Goal")
        
        
        plt.xticks(np.arange(0, 20, 1))  # X-axis ticks every 2 units
        plt.yticks(np.arange(0, 20, 1)) 
        plt.grid(True)
        
        plt.legend(fontsize=12)
        plt.title("A* algorithm")
        plt.show()