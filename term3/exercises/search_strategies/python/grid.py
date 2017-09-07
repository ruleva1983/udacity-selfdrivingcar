import numpy as np


class Grid(object):
    def __init__(self, shape=(1,1), blocks=None):
        self.shape = shape
        self.grid = np.zeros(self.shape)
        self.actions = ["U", "D", "L", "R"]
        self.cost = 1

        if blocks is not None:
            self._set_blocks(blocks)

    def _set_blocks(self, blocks):
        if blocks is list:
            for b in blocks:
                self.grid[b] = 1
        else:
            self.grid += blocks

    def apply_action(self, init_state, action):
        if action == "D":
            res = init_state[1] + 1
            return np.array([init_state[0], res]) if res < self.shape[1] else None
        elif action == "U":
            res = init_state[1] - 1
            return np.array([init_state[0], res]) if res >= 0 else None
        elif action == "L":
            res = init_state[0] - 1
            return np.array([res, init_state[1]]) if res >= 0 else None
        elif action == "R":
            res = init_state[0] + 1
            return np.array([res, init_state[1]]) if res < self.shape[0] else None
        else:
            print ("Wrong action chosen")
            raise ValueError


    def find_successors(self, state):
        successors = list()
        for a in self.actions:
            res = self.apply_action(state, a)
            if res is not None:
                successors.append(res)
        return list(successors)



