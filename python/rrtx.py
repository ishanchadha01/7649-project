from random import random
import math
import PartiallyObservablePlanner
from queue import PriorityQueue
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

    self.rrt_tree = [] # tree to calculate path to goal
    self.spatial_hash = {[] for _ in range(math.prod(self.world.dims))} # spatial hashmap flattened for d dims
    self.pq = PriorityQueue()
    self.n = 1000
    self.d = len(self.world.dims)
    self.radius = math.floor((math.prod(self.world.dims) * math.log(self.n) / self.n) ** (1/self.d)) # radius of ball for neighbors


  def sample(self, goal_node):
    if random.random() < self.eps: # some % chance of returning goal node
      return goal_node
    # return uniform floating point position within the world
    return self.world.random_position()


  def nearest(self, node, k):
    dist,idx = self.tree.query([node], k)
    return dist, idx


  def extract_path(self, start, goal):
    path = [goal]
    curr = goal
    while curr != start:
      path.append(curr.parent)
      curr = curr.parent
    return path.reverse()


  def observe_world(self):
    if self.world.obstacle_detected():
      pass
    if self.world.obstacle_vanished():
      pass


  def sample_free(self, x_goal):
    pass


  def steer(self):
    pass


  def obstacle_free(self):
    pass


  def build_rrt_tree(self, start, goal):
    for _ in range(self.iters):
      x_rand = self.sample_free(goal)
      x_nearest = self.nearest(self.rrt_tree, x_rand)
      x_new = self.steer(x_nearest, x_rand) # includes step function

      if self.obstacle_free(x_nearest, x_new):
        self.rrt_tree.append(x_new)
        x_new.parent = x_nearest
        x_nearest.children.append(x_new)

        # add new node to spatial hashmap
        hash_idx = math.floor(x_new.coord[-1])
        for dim in range(len(self.world.dims), 1 ,-1):
          hash_idx = math.floor(x_new.coord[dim-1]) + self.world.dims[dim] * hash_idx
        self.spatial_hash[hash_idx].append(x_new)

    return self.rrt_tree


  def run(self) -> None:
    # construct tree backwards from goal to current node
    root_node = Node(self.x_goal)
    curr_node = Node(self.curr_pos)
    self.rrt_tree = self.build_rrt_tree(root_node, curr_node)
    while curr_node != self.x_goal:
      self.planned_path = self.extract_path(root_node, curr_node)
      next_node = self.step()
      self.observe_world()
      
  
  def step(self) -> Coord:
    if len(self.planned_path) == 0:
      return self.curr_pos
    return self.planned_path.pop()