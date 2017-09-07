import numpy as np


class Search(object):
    """
    A base class for all searching algorithms on grid
    """
    def __init__(self, grid, start_point, final_point):
        self.grid = grid
        self.start_point = start_point
        self.final_point = final_point


class dijkstra(Search):
    """
    A specific class that applies dijkstra shortest path algorithm
    """
    def __init__(self, grid, start_point, final_point):
        Search.__init__(self, grid, start_point, final_point)

    def eval_cost(self):
        """
        This function evaluates the cost of the shortest path
        :return: The cost of the shortest path
        """
        if self.start_point == self.final_point:
            total_cost = 0
            return total_cost

        open_list = [np.append([0], self.start_point)]
        closed_list = np.copy(self.grid.grid)

        while True:
            if len(open_list) == 0:
                print ("There is no open path from the origin to the final destination")
                return None

            open_list = sorted(open_list, key=lambda row: row[0], reverse=True)
            element = open_list.pop()

            if (element[1] == self.final_point[0]) and (element[2] == self.final_point[1]):
                total_cost = element[0]
                print ("Final point reached with cost {}".format(total_cost))
                print(closed_list)
                return total_cost

            successors = self.grid.find_successors(element[1:])
            open_list += [[element[0] + self.grid.cost] + list(s) for s in successors if closed_list[s[0], s[1]] == 0]
            closed_list[element[1], element[2]] = 1

class Astar(Search):
    def __init__(self, grid, start_point, final_point, heuristic = None):
        Search.__init__(self, grid, start_point, final_point)
        if heuristic is not None:
            self.heuristic = heuristic
        else:
            self.heuristic = self._default_heuristic()

    def eval_cost(self):
        if self.start_point == self.final_point:
            total_cost = 0
            return total_cost

        x = self.start_point[0]
        y = self.start_point[1]
        f = self.heuristic[x,y]
        open_list = [[f,x,y]]
        closed_list = np.copy(self.grid.grid)

        while True:
            if len(open_list) == 0:
                print ("There is no open path from the origin to the final destination")
                return None

            open_list = sorted(open_list, key=lambda row: row[0], reverse=True)
            element = open_list.pop()

            x = element[1]
            y = element[2]
            f = element[0]

            if (x == self.final_point[0]) and (y == self.final_point[1]):
                total_cost = f - self.heuristic[x,y]
                print ("Final point reached with cost {}".format(total_cost))
                print (closed_list)
                return total_cost

            successors = self.grid.find_successors(element[1:])
            open_list += [[f + 1 + self.heuristic[s[0], s[1]] - self.heuristic[x, y], s[0], s[1]]
                          for s in successors if closed_list[s[0], s[1]] == 0]

            closed_list[element[1], element[2]] = 1

    def _default_heuristic(self):
        grid_shape = self.grid.grid.shape

        heuristic = np.zeros(grid_shape)

        for i in range(grid_shape[0]):
            for j in range(grid_shape[1]):
                heuristic[i,j] = abs(self.final_point[0] - i) + abs(self.final_point[0] - j)
        return heuristic
