def visualize_p(maze, path):
    for aa in path:
        x, y = aa
        maze[x][y] = "*"
    for i in range(10):
        for j in range(10):
            print(maze[i][j], end=" ")
        print()
    print(sep="")
    return None


def visualize_m(maze, MOB_list):
    for elem in MOB_list:
        x, y = elem.position
        maze[x][y] = 2
    for i in range(10):
        for j in range(10):
            print(maze[i][j], end=" ")
        print()
    print(sep="")
    return None
