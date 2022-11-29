import numpy as np


class Obstacle:

    def __init__(self, cur, name=None):
        self.current = cur
        self.name = name

    def random_moving(self, origin_maze):
        height = len(origin_maze)
        width = len(origin_maze[0])
        if np.random.randint(0, 3) == 1:
            dim = np.random.randint(1, 3)
            if dim == 1:
                direction = np.random.randint(1, 3)
                if direction == 1 and (self.current[0] < height - 1) and origin_maze[self.current[0] + 1][self.current[1]] != 1:
                    origin_maze[self.current[0]][self.current[1]] = 0
                    self.current[0] += 1
                    origin_maze[self.current[0]][self.current[1]] = 2
                elif (1 < self.current[0]) and origin_maze[self.current[0] - 1][self.current[1]] != 1:
                    origin_maze[self.current[0]][self.current[1]] = 0
                    self.current[0] -= 1
                    origin_maze[self.current[0]][self.current[1]] = 2
            else:
                direction = np.random.randint(1, 3)
                if direction == 1 and (self.current[1] < width - 1) and origin_maze[self.current[0]][self.current[1] + 1] != 1:
                    origin_maze[self.current[0]][self.current[1]] = 0
                    self.current[1] += 1
                    origin_maze[self.current[0]][self.current[1]] = 2
                elif (1 < self.current[1]) and origin_maze[self.current[0]][self.current[1] - 1] != 1:
                    origin_maze[self.current[0]][self.current[1]] = 0
                    self.current[1] -= 1
                    origin_maze[self.current[0]][self.current[1]] = 2
        return origin_maze
