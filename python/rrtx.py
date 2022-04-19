from random import random
from sklearn.neighbors import KDTree
import numpy as np
import PartiallyObservablePlanner
from main.node import Node
from main.world import World, Coord


class RRTX(PartiallyObservablePlanner):

  def __init__(self, *args, **kwargs) -> None:
    super().__init__(*args, **kwargs)
    self.iters = kwargs.get('iters', 100)
    self.MIN_EXPLORATION_DISTANCE = kwargs.get('MIN_EXPLORATION_DISTANCE', 0)
    self.MAX_EXPLORATION_DISTANCE = kwargs.get('MAX_EXPLORATION_DISTANCE', 10)
    self.eps = kwargs.get('eps', 0.1)
    self.planned_path = []
    self.rrt_tree = []
    self.kd_tree = KDTree([])

  def sample(self, goal_node):
    if random.random() < self.eps: # some % chance of returning goal node
      return goal_node
    # return uniform floating point position within the world
    return self.world.random_position()

  def nearest(self, node, k):
    dist,idx =  self.tree.query([node], k)
    return dist, idx

  def sample_free(self, x_goal):
    pass

  def steer(self):
    pass

  def obstacle_free(self):
    pass

  def build_rrt_tree(x_start, x_goal):
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

  def plan(self) -> None:
    # construct tree backwards from goal to current node
    root_node = Node(self.x_goal)
    curr_node = Node(self.curr_pos)
    tree = self.build_rrt_tree(root_node, curr_node)
    self.planned_path = self.extract_path(tree, root_node, curr_node)
  
  def step(self) -> Coord:
    if len(self.planned_path) == 0:
      return self.curr_pos
    return self.planned_path.pop()