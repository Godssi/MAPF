import numpy as np
import time
from agent import Agent
from moving_obstacle import *
from a_star import aStar
from map_gen import *
from plot_map_func import *


def main():
    start_time = check_time = time.time()
    np.set_printoptions(linewidth=np.inf)

    # new_maze = random_maze_gen()
    new_maze = test_maze_gen_2()
    potential_map = potential_map_generator(new_maze)
    print("map_gen time:", time.time() - start_time, " (s)")
    plot_map_2d(potential_map)
    print("end:", len(new_maze)-2, len(new_maze[0])-2)
    Agent_A = Agent((1, 1), [1, 1], (len(new_maze)-2, len(new_maze[0])-2), "A")
    Agent_A.set_astar_path(new_maze, potential_map, 'init')
    plot_origin_map_2d(new_maze, Agent_A)
    print(f"{1}-step time:", time.time() - check_time, " (s)")

    Obstacle_B = Obstacle([6, 12], "B")
    Obstacle_C = Obstacle([11, 7], "C")
    flag = True
    i = 1
    while flag:
        new_maze = Obstacle_B.random_moving(new_maze)
        new_maze = Obstacle_C.random_moving(new_maze)
        potential_map2 = modify_potential_map(new_maze, potential_map)
        Agent_A.set_astar_path(new_maze, potential_map2, 'renew')
        plot_origin_map_2d(new_maze, Agent_A)
        flag = Agent_A.move_path()
        i = i + 1
        print(f"{i + 1}-step time:", time.time() - check_time, " (s)")
        print(flag)
        check_time = time.time()

    print("end time:", time.time() - start_time, " (s)")


if __name__ == '__main__':
    main()
