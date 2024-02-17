from src.maze_solver.maze_solver import MazeSolver


def main():
    maze_paths = [
        # "images/mazes/maze1.bmp",
        # images/mazes/maze2.bmp",
        "images/mazes/maze3.bmp",
        # images/mazes/maze4.bmp",
    ]

    matrix_size = 30
    family = "BFS"
    solver = MazeSolver(matrix_size=matrix_size, family=family)

    for maze in maze_paths:
        solver.set_maze(maze)
        solver.solve()
        solver.display_maze("images/output/maze.bmp")


if __name__ == "__main__":
    main()
