import math
from PIL import Image
import numpy as np
from maze_solver.colors import Colors
from maze_solver.node import Node
from queue import PriorityQueue

class MazeSolver:
    def __init__(self, path: str = None, matrix_size: int = 80, family: str = "BFS"):
        self.path = path
        self.matrix_size = matrix_size
        self.matrix = None
        self.start_position = None
        self.goals = None
        self.start_node = None
        self.goal_nodes = set()
        self.goal_positions = set()
        self.family = family
        self.counter = 0 

    def set_maze(self, maze_path: str):
        """
        Set the maze to solve
        :param maze_path: The path to the maze image
        :return:
        """
        self.path = maze_path
        self.matrix = self._discretize_maze()
        self.graph = self._create_graph()

    def _discretize_maze(self):
        """
        Discretize the maze image into a matrix
        :return: The discrete maze
        """
        # Create a pixelated version of the maze
        pixelated_maze = self._pixelate()

        # Resize the pixelated maze to be a multiple of the matrix size
        new_width = pixelated_maze.width - (pixelated_maze.width % self.matrix_size)
        new_height = pixelated_maze.height - (pixelated_maze.height % self.matrix_size)
        pixelated_maze = pixelated_maze.resize((new_width, new_height))

        # Set the chunk size to be scanned
        chunk_width = pixelated_maze.width // self.matrix_size
        chunk_height = pixelated_maze.height // self.matrix_size

        # Create a matrix to store the maze
        matrix = np.zeros((self.matrix_size, self.matrix_size), dtype=int)

        # Scan the pixelated maze and set the matrix values
        self.start_found = False
        for y in range(self.matrix_size):
            for x in range(self.matrix_size):
                chunk = pixelated_maze.crop(
                    (x * chunk_width, y * chunk_height, (x + 1) * chunk_width, (y + 1) * chunk_height))
                matrix[y, x] = self._get_color(chunk)

        del self.start_found
        return matrix

    def _pixelate(self):
        """
        Pixelate the maze image
        :return:
        """
        img = Image.open(self.path)

        # Resize smoothly down to 16x16 pixels
        img_small = img.resize((self.matrix_size, self.matrix_size), resample=Image.Resampling.BILINEAR)

        # Scale back up using NEAREST to original size
        result = img_small.resize(img.size, Image.Resampling.NEAREST)

        # remove pixels that are not pure black, white, green or red
        result = result.point(lambda p: p > 128 and 255)

        return result
    
    def _get_color(self, chunk: Image):
        """
        Get the color of the chunk
        :param chunk: The chunk to get the color from
        :return: The color of the chunk
        """
        # Convert the chunk to a numpy array
        chunk = np.array(chunk)

        # Get the unique colors in the chunk
        unique, counts = np.unique(chunk.reshape(-1, chunk.shape[2]), axis=0, return_counts=True)

        # Get the most frequent color
        color = unique[np.argmax(counts)]

        # Check if the color is black, white, green or red
        if np.array_equal(color, [0, 0, 0]):  # black
            return Colors.WALL.value
        elif np.array_equal(color, [255, 255, 255]):  # white
            return Colors.PATH.value
        elif np.array_equal(color, [255, 0, 0]) and not self.start_found:  # red
            self.start_found = True
            return Colors.START.value
        elif np.array_equal(color, [0, 255, 0]):  # green
            return Colors.END.value
        else:
            return Colors.PATH.value

    def _create_graph(self):
        """
        Create a graph from the matrix
        :return: The graph
        """
        graph = {}

        # Add all available nodes to the graph (excluding walls)
        for y in range(self.matrix_size):
            for x in range(self.matrix_size):
                if self.matrix[y][x] != Colors.WALL.value:
                    if self.matrix[y][x] == Colors.START.value:
                        node = Node(x, y, is_start=True)
                        graph[(x, y)] = node
                        self.start_node = node
                        self.start_position = (x, y)
                    elif self.matrix[y][x] == Colors.END.value:
                        node = Node(x, y, is_goal=True)
                        graph[(x, y)] = node
                        self.goal_nodes.add(node)
                        self.goal_positions.add((x, y))
                    else:
                        node = Node(x, y)
                        graph[(x, y)] = node

        # Add neighbors to each node
        for node in graph.values():
            x, y = node.position
            if y > 0 and self.matrix[y - 1][x] != Colors.WALL.value:
                node.neighbors.append(graph[(x, y - 1)])
            if y < self.matrix_size - 1 and self.matrix[y + 1][x] != Colors.WALL.value:
                node.neighbors.append(graph[(x, y + 1)])
            if x > 0 and self.matrix[y][x - 1] != Colors.WALL.value:
                node.neighbors.append(graph[(x - 1, y)])
            if x < self.matrix_size - 1 and self.matrix[y][x + 1] != Colors.WALL.value:
                node.neighbors.append(graph[(x + 1, y)])

        return graph

    def solve(self):
        if self.family == "A*":
            return self.solve_a_star()
        elif self.family == "BFS":
            return self.solve_bfs()
        elif self.family == "DFS":
            return self.solve_dfs()
        else:
            raise ValueError(f"Unsupported family: {self.family}")
        
    def solve_a_star(self):
        """
        Solve the maze using the A* search algorithm.
        """
        open_set = PriorityQueue()
        self.counter = 0
        # start node with priority 0
        open_set.put((0, self.counter, self.start_node))  
        came_from = {} 
        g_score = {node: float("inf") for node in self.graph.values()}
        
         # the cost of the start node to itself is 0
        g_score[self.start_node] = 0 
        f_score = {node: float("inf") for node in self.graph.values()}
        
        # estimated cost from start to goal
        f_score[self.start_node] = self._heuristic(self.start_node.position, self.goal_positions)  

        while not open_set.empty():
            # get the node with the lowest f-score
            current = open_set.get()[2]  

            if current in self.goal_nodes:
                return self._reconstruct_path(came_from, current)

            for neighbor in current.neighbors:
                # assuming each step costs 1
                tentative_g_score = g_score[current] + 1  
                
                # in case this path to neighbor is better than any previously recorded
                if tentative_g_score < g_score[neighbor]:  
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = g_score[neighbor] + self._heuristic(neighbor.position, self.goal_positions)
                    
                    # add the neighbor if it's not in the open set
                    if not any(neighbor == item[2] for item in open_set.queue):  
                        self.counter += 1
                        open_set.put((f_score[neighbor], self.counter, neighbor))

        return False
    
    def solve_bfs(self):
        pass

    def _heuristic(self, start, goals):
        """
        Compute the heuristic cost from start to each goal and return the smallest.
        It can switch between Manhattan and Euclidean by changing the return.
        """
        # manhattan Distance
        manhattan_distance = min(abs(start[0] - goal[0]) + abs(start[1] - goal[1]) for goal in goals)

        # euclidean Distance
        euclidean_distance = min(math.sqrt((start[0] - goal[0])**2 + (start[1] - goal[1])**2) for goal in goals)

        # heuristic return
        return manhattan_distance  
    
    def _reconstruct_path(self, came_from, current):
        """
        Reconstruct the path from the start node to the goal node and mark it in the matrix.
        """
        total_path = [current]
        while current in came_from:
            current = came_from[current]
            total_path.append(current)
        total_path.reverse()

        # mark the path in the matrix
        for node in total_path:
            if node != self.start_node and node not in self.goal_nodes:
                self.matrix[node.position[1]][node.position[0]] = Colors.SOLUTION.value

        return total_path
    
    def display_original_maze(self, output_path: str = "images/output/maze.bmp"):
        """
        Display the pixelated version of the maze without the solution.
        """
        colors = {
            Colors.PATH.value: (255, 255, 255),  # white for paths
            Colors.WALL.value: (0, 0, 0),  # black for walls
            Colors.START.value: (0, 255, 0),  # green for start
            Colors.END.value: (255, 0, 0),  # red for end
        }

        # maze layout representation
        image = Image.new("RGB", (self.matrix.shape[1], self.matrix.shape[0]))
        for y in range(self.matrix.shape[0]):
            for x in range(self.matrix.shape[1]):
                image.putpixel((x, y), colors.get(self.matrix[y, x], (255, 255, 255)))

        # save the original maze image before solving
        image.save(output_path)

    def display_maze(self, output_path: str = "images/output/solution.bmp"):
        """
        Display the matrix of the maze in a bitmap image
        :return:
        """
        colors = {
            Colors.PATH.value: (255, 255, 255),  # white for paths
            Colors.WALL.value: (0, 0, 0),  # black for walls
            Colors.START.value: (0, 255, 0),  # green for start
            Colors.END.value: (255, 0, 0),  # red for end
            Colors.SOLUTION.value: (0, 0, 255),  # blue for solution
        }

        # create a new image with the same dimensions as the matrix
        image = Image.new("RGB", (self.matrix.shape[1], self.matrix.shape[0]))

        # set color for each pixel based on the matrix values
        for y in range(self.matrix.shape[0]):
            for x in range(self.matrix.shape[1]):
                image.putpixel((x, y), colors[self.matrix[y, x]])

        # save the image
        image.save(output_path)