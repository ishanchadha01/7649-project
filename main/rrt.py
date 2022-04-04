from math import sqrt
import random
from main.PartiallyObservablePlanner import PartiallyObservablePlanner
from main.world import Coord
from node import Node
from typing import Callable, List, Tuple, TypeVar

Tree = List[Node]

class RRT(PartiallyObservablePlanner):
  def __init__(self, *args, **kwargs) -> None:
    super().__init__(*args, **kwargs)
    self.iters = kwargs.get('iters', 100)
    self.MIN_EXPLORATION_DISTANCE = kwargs.get('MIN_EXPLORATION_DISTANCE', 0)
    self.MAX_EXPLORATION_DISTANCE = kwargs.get('MAX_EXPLORATION_DISTANCE', 10)
    self.eps = kwargs.get('eps', 0.1)
    self.planned_path = []

  def sample(self, goal_node):
    if random.random() < self.eps: # some % chance of returning goal node
      return goal_node
    # return uniform floating point position within the world
    return self.world.random_position()

  def nearest(self, tree, rand):
    min_dist = float('inf')
    nearest_node = None
    for node in tree:
        dist = (node[0] - rand[0]) ** 2 + (node[1] - rand[1]) ** 2
        if dist < min_dist:
            min_dist = dist
            nearest_node = node
    return nearest_node

  def steer(self, nearest, rand):
    vec = (rand[0] - nearest[0], rand[1] - nearest[1])
    dist = sqrt(vec[0]**2 + vec[1]**2)
    # rand == nearest happens almost only when both are goal node (because random floats rarely equal each other)
    if dist == 0:
        return nearest
    # if the new node is too close, push it further to ensure useful exploration (min = 0 is best anyways based on testing)
    if dist < self.MIN_EXPLORATION_DISTANCE:
        scale = sqrt(self.MIN_EXPLORATION_DISTANCE / dist)
        vec = (vec[0] * scale, vec[1] * scale)
    # if the new node is too far, pull it closer to ensure useful exploration (max = 10 is best anyways based on testing)
    # lower max distance caused the paths to be significantly longer with many small jumps instead of a couple big steps towards the goal
    if dist > self.MAX_EXPLORATION_DISTANCE:
        scale = sqrt(self.MAX_EXPLORATION_DISTANCE / dist)
        vec = (vec[0] * scale, vec[1] * scale)
    # return the new node which has been projected closer/further from the nearest node
    return (nearest[0] + vec[0], nearest[1] + vec[1])

  def obstacle_free(self, nearest, new):
    # helper function to check if a node is in the obstacle
    def lines_intersect(line1, line2):
        # line intersection algorithm taken from https://stackoverflow.com/a/4977569
        (x00, y00), (x01, y01) = line1
        (x10, y10), (x11, y11) = line2
        d = x11*y01 - x01*y11 # determinant of the matrix [[x11 x01], [y11 y01]]
        if d == 0: # parallel lines
            return False
        s = ((x00 - x10) * y01 - (y00 - y10) * x01) / d
        t = -(-(x00 - x10) * y11 + (y00 - y10) * x11) / d
        if 0 <= s <= 1 and 0 <= t <= 1: # intersection logic taken from https://stackoverflow.com/a/4977569
            return True
        return False
    # check if the path from the nearest node to the new node intersects with any of the obstacle edges
    for edge in self.world.OBSTACLE_EDGES:
        if lines_intersect(edge, (nearest, new)):
            # the path intersects with an obstacle edge, so it is not a valid path
            return False
    return True

  def built_rrt_tree(self, x_start: Node, x_goal: Node) -> Tree:
    tree: Tree = [x_start]
    for _ in range(self.iters):
      x_rand = self.sample_free(x_goal)
      x_nearest = self.nearest(tree, x_rand)
      x_new = self.steer(x_nearest, x_rand)
      tree.append(x_new)
      if self.obstacle_free(x_nearest, x_new):
        tree.append(x_new)
        x_new.parent = x_nearest
        x_nearest.children.append(x_new)
    return tree

  def extract_path(self, tree: Tree, goal: Node) -> None:
    path = [goal]
    curr = goal
    while curr != self.x_start:
      path.append(curr.parent)
      curr = curr.parent
    return path.reverse()

  def plan(self) -> None:
    start = Node(self.curr_pos)
    goal = Node(self.x_goal)
    tree = self.built_rrt_tree(start, goal)
    self.planned_path = self.extract_path(tree, start, goal)
  
  def step(self) -> Coord:
    if len(self.planned_path) == 0:
      return self.curr_pos
    return self.planned_path.pop()

