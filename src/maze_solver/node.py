from typing import Tuple


class Node:
    def __init__(self, x: int, y: int, is_start: bool = False, is_goal: bool = False):
        self.neighbors = []
        self.position = (x, y)
        self.is_start = is_start
        self.is_goal = is_goal

    def __repr__(self):
        return f"Node({self.position})"

    def __str__(self):
        return f"Node({self.position})"

    def __eq__(self, other):
        return self.position == other.position

    def __hash__(self):
        return hash(self.position)
