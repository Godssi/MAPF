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

    def set_astar_path(self, new_maze, potential_map, type):
        if type == 'renew':
            self.path = aStar(potential_map, self.current, self.goal, new_maze)
        elif type == 'init':
            self.path = aStar(potential_map, self.start, self.goal, new_maze)

    def move_path(self):
        if len(self.path) != 2:
            print(len(self.path))
            print(self.current)
            self.current = self.path[-2]
            return True
        return False
