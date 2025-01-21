from A_star_module import Node
from A_star_module import Path
import matplotlib.pyplot as plt
import numpy as np


grid = np.zeros((20,20))

grid[5:10, 15] = 1
grid[10, 5:16] = 1

start_pos = (3,5)
goal_pos = (18, 18)

# bug start and gaol pos can not be on the obstacles
# needed to be fixed 
# start_pos = (2,5)
# goal_pos = (10, 13)

path = Path(grid, start_pos, goal_pos)

result = path.find_path()
path.visualize_path(result)