from maze_solver.maze_solver import MazeSolver

def main():
    maze_paths = [
        "images/mazes/maze1.bmp"
        # images/mazes/maze2.bmp",
        #"images/mazes/maze3.bmp",
        # images/mazes/maze4.bmp",
    ]

    matrix_size = 150
    
    # list of algorithms to try
    families = ["A*", "BFS", "DFS"]  
    
    family = "A*"
    solver = MazeSolver(matrix_size=matrix_size, family=family)

    for maze in maze_paths:
        solver.set_maze(maze)
        solver.solve()
        solver.display_original_maze()
        solver.display_maze()
        
if __name__ == "__main__":
    main()