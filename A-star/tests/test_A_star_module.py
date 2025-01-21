from src.A_star_module import Path
import pytest
import numpy as np
from math import sqrt

grid = np.zeros((20,20))
grid[5:10, 15] = 1
grid[10, 5:16] = 1

@pytest.fixture
def path():
    return Path(grid, (2,2),(17,18))

def test_calculate_heuristic(path):
    assert path.calculate_heuristic((1,1), (1,2)) == 1
    assert path.calculate_heuristic((2,1), (10,18)) == sqrt((10-2)**2 + (18-1)**2)