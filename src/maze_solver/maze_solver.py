from PIL import Image
from src.maze_solver.colors import Colors
from src.maze_solver.node import Node
import numpy as np


class MazeSolver:
    def __init__(self, path: str = None, matrix_size: int = 80, family: str = "BFS"):
        self.path = path
        self.matrix_size = matrix_size
        self.matrix = None
        self.start_position = None
        self.goals = None
        self.start_node = None
        self.start_position = None
        self.goal_nodes = set()
        self.goal_positions = set()
        self.family = family

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
        """
        Solve the maze
        :return:
        """
        pass

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

        # Create a new image with the same dimensions as the matrix
        image = Image.new("RGB", (self.matrix.shape[1], self.matrix.shape[0]))

        # Set color for each pixel based on the matrix values
        for y in range(self.matrix.shape[0]):
            for x in range(self.matrix.shape[1]):
                image.putpixel((x, y), colors[self.matrix[y, x]])

        # Save the image
        image.save(output_path)