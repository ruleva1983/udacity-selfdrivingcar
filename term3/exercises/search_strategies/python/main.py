import grid
import search

import numpy as np

if __name__ == "__main__":

    shape = (7,7)
    blocks = np.zeros(shape)
    blocks[0, 1] = 1
    blocks[0, 2] = 1
    blocks[1, 2] = 1
    blocks[1, 3] = 1
    blocks[3, 2] = 1
    blocks[3, 3] = 1
    blocks[4, 2] = 1


    Grid = grid.Grid(shape=shape, blocks=blocks)
    s = search.dijkstra(Grid, [0,0], [6,6])
    s.eval_cost()

    s = search.Astar(Grid, [0,0], [6,6])
    s.eval_cost()