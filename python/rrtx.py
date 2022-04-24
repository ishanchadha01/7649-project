import random
import math
import tkinter as tk
import numpy as np
from PartiallyObservablePlanner import PartiallyObservablePlanner
from queue import PriorityQueue
from node import Node
from world import World, Coord


class RRTX(PartiallyObservablePlanner):

  def __init__(self, gui, *args, **kwargs) -> None:
    super().__init__(*args, **kwargs)
    self.iters = kwargs.get('iters', 1000)
    self.MIN_EXPLORATION_DISTANCE = kwargs.get('MIN_EXPLORATION_DISTANCE', 0)
    self.MAX_EXPLORATION_DISTANCE = kwargs.get('MAX_EXPLORATION_DISTANCE', 10)
    self.eps = kwargs.get('eps', 0.1)
    self.planned_path = []

    self.rrt_tree = [] # tree to calculate path to goal
    self.step_size = 0.1
    self.spatial_hash = [[] for _ in range(math.prod(self.world.dims))] # spatial hashmap flattened for d dims
    self.pq = PriorityQueue()
    self.n = 1000
    self.d = len(self.world.dims)
    self.radius = math.floor((math.prod(self.world.dims) * math.log(self.n) / self.n) ** (1/self.d)) # radius of ball for neighbors

    self.gui = kwargs.get('gui', True)


  def dist(self, x1, x2):
    return np.linalg.norm(np.array(x2.coord) - np.array(x1.coord))


  def spatial_hash_add(self, node):
    # add node to spatial hash
    hash_idx = math.floor(node.coord[-1])
    for dim in range(len(self.world.dims)-1, 1, -1):
      hash_idx = math.floor(node.coord[dim-1]) + self.world.dims[dim] * hash_idx
    self.spatial_hash[hash_idx].append(node)


  def spatial_hash_get(self, node):
    # get nearest node from spatial hash approximately
    hash_idx = math.floor(node.coord[-1])
    for dim in range(len(self.world.dims)-1, 1, -1):
      hash_idx = math.floor(node.coord[dim-1]) + self.world.dims[dim] * hash_idx
    bin_at_dist = 0

    # search current bin and neighboring bins until neighbor found or out of bounds
    while True:
      try:
        if len(self.spatial_hash[hash_idx + bin_at_dist]) > 0:
          return self.spatial_hash[hash_idx + bin_at_dist][-1]
        if len(self.spatial_hash[hash_idx + bin_at_dist]) > 0:
          return self.spatial_hash[hash_idx - bin_at_dist][-1]
        bin_at_dist += 1
      except:
        return self.sample(self.x_start)



  def sample(self, goal_node):
    if random.random() < self.eps: # some % chance of returning goal node
      return goal_node
    # return uniform floating point position within the world
    return Node(self.world.random_position())


  def extract_path(self, start):
    curr = self.rrt_tree[-1]
    path = [curr]
    while curr != start:
      print(curr.coord, start.coord)
      path.append(curr.parent)
      curr = curr.parent
    return path


  def observe_world(self):
    if self.world.obstacle_detected():
      self.reduce_inconsistency()
    if self.world.obstacle_vanished():
      self.propagate_changes()
      self.pq.put()


  def steer(self, x1, x2):
    coord = tuple([self.step_size * (x2_d - x1_d) + x1_d for x1_d, x2_d in zip(x1.coord, x2.coord)])
    return Node(coord=coord)


  def obstacle_free(self, x1, x2):
    return True


  def build_rrt_tree(self, start, goal):
    x_rand = self.sample(goal)
    x_new = self.steer(start, x_rand) # includes step function
    if self.obstacle_free(x_rand, x_new):
      self.rrt_tree.append(x_new)
      x_new.parent = start
      start.children.append(x_new)
    while self.dist(x_new, goal) >= self.eps:
      x_rand = self.sample(goal)
      x_nearest = self.spatial_hash_get(x_rand) # use spatial hash to find nearest neighbor approximately 
      x_new = self.steer(x_nearest, x_rand)

      if self.obstacle_free(x_rand, x_new):
        self.rrt_tree.append(x_new)
        x_new.parent = x_rand
        x_rand.children.append(x_new)
        self.spatial_hash_add(x_new) # add new node to spatial hashmap
        
    return self.rrt_tree


  def run(self) -> None:
    # construct tree backwards from goal to current node
    root_node = self.x_goal
    curr_node = self.curr_pos
    self.rrt_tree = self.build_rrt_tree(root_node, curr_node)
    self.planned_path = self.extract_path(curr_node)
    step = 0
    while curr_node != self.x_goal:
      next_node = self.step()
      self.observe_world()

      if self.gui:
        self.render(step)
      step += 1
      
  
  def step(self) -> Coord:
    if len(self.planned_path) == 0:
      return self.curr_pos
    return self.planned_path.pop()

  def render(self, step):
    window = tk.Tk()

    # create canvas to plot rrt graph
    canvas = tk.Canvas(window, bg="white", height=300, width=300)
    python_green = "#476042"
    canvas.create_oval(5, 5, 5, 5, fill=python_green)

    # create buttons at the bottom to step through simulation
    window.rowconfigure([0,1], minsize=50, weight=1)
    window.columnconfigure([0,1], minsize=50, weight=1)
    lbl_value = tk.Label(master=window, text=f"Step: {step}")
    lbl_value.grid(row=1, column=0)
    clicked = tk.BooleanVar(False)
    step_btn = tk.Button(master=window, text="Next", command=lambda: clicked.set(True))
    step_btn.grid(row=1, column=1, sticky="nsew")
    window.wait_variable(clicked)


if __name__=='__main__':
  world = World()
  rrtx = RRTX(world=world, x_start=Node((5,5)), x_goal=Node((85,85)), gui=True)
  rrtx.run()