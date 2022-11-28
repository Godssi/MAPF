from a_star import aStar


class Agent:

    def __init__(self, start, cur, goal, name=None, path=None):
        self.start = start
        self.current = cur
        self.goal = goal
        self.name = name
        self.path = path

    def set_path(self, path):
        self.path = path

    def set_astar_path(self, new_maze, potential_map):
        self.path = aStar(potential_map, self.start, self.goal, new_maze)