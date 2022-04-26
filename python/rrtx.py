from copy import deepcopy
import random
import math
import tkinter as tk
import numpy as np
from scipy.spatial import KDTree
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
    self.eps = kwargs.get('eps', .01)
    self.planned_path = []

    self.thresh = 5
    self.rrt_tree = [] # tree to calculate path to goal
    self.step_size = 1
    self.spatial_hash = [[[] for _ in range(self.world.dims[0])] for _ in range(self.world.dims[1])] # spatial hashmap flattened for d dims
    self.pq = PriorityQueue()
    self.n = 1000
    self.radius = math.floor((math.prod(self.world.dims) * math.log(self.n) / self.n) ** (1/2)) # radius of ball for neighbors

    self.gui = kwargs.get('gui', True)


  def dist(self, x1, x2):
    return np.linalg.norm(np.array(x2.coord) - np.array(x1.coord))


  def spatial_hash_add(self, node):
    # add node to spatial hash
    x = math.floor(node.coord[0])
    y = math.floor(node.coord[1])
    print(node.coord, x, y, np.array(self.spatial_hash).shape)
    self.spatial_hash[x][y].append(node)


  def spatial_hash_get(self, node, fallback):
    min_dist = float('inf')
    min_node = deepcopy(fallback)
    for node2 in self.rrt_tree:
      if self.dist(node, node2) < min_dist:
        min_dist = self.dist(node, node2)
        min_node = node2
    return min_node
    # # get nearest node from spatial hash approximately
    # x = math.floor(node.coord[0])
    # y = math.floor(node.coord[1])
    # direction = [(1,0), (0,1), (-1,0), (0,-1)]
    # dist = 0

    # # search current bin and neighboring bins until neighbor found or out of bounds
    # i = 0
    # while True:
    #   print((x,y))
    #   i+=1
    #   if i%2 == 0:
    #     dist += 1
    #   for d in range(0, dist):
    #     x += direction[i%4][0]
    #     y += direction[i%4][1]
    #   try:
    #     if len(self.spatial_hash[x][y]) > 0:
    #       return random.choice(self.spatial_hash[x][y])
    #   except:
    #     min_dist = float('inf')
    #     node_node = deepcopy(fallback)
    #     for node2 in self.rrt_tree:
    #       if self.dist(node, node2) < min_dist:
    #         min_dist = self.dist(node, node2)
    #         min_node = node2
    #     return(min_node)


  def sample(self, goal_node):
    if random.random() < self.eps: # some % chance of returning goal node
      return goal_node
    # return uniform floating point position within the world
    return Node(self.world.random_position())


  def extract_path(self, start):
    curr = self.rrt_tree[-1]
    path = [curr]
    while curr.coord != start.coord:
      path.append(curr.parent)
      curr = curr.parent
    return list(reversed(path))


  def observe_world(self):
    if self.world.obstacle_detected():
      self.reduce_inconsistency()
    if self.world.obstacle_vanished():
      self.propagate_changes()
      self.pq.put()


  def steer(self, x1, x2):
    # factor = self.step_size / self.dist(x1, x2)
    # coord = tuple([factor * (x2_d - x1_d) + x1_d for x1_d, x2_d in zip(x1.coord, x2.coord)])
    # return Node(coord=coord)
    return x2


  def obstacle_free(self, x1, x2):
    def lines_intersect(line1, line2):
      (x00, y00), (x01, y01) = line1
      (x10, y10), (x11, y11) = line2
      d = x11*y01 - x01*y11 # determinant of the matrix [[x11 x01], [y11 y01]]
      if d == 0: # parallel lines
        return False
      s = ((x00 - x10) * y01 - (y00 - y10) * x01) / d
      t = -(-(x00 - x10) * y11 + (y00 - y10) * x11) / d
      if 0 <= s <= 1 and 0 <= t <= 1:
          return True
      return False

    for edge in self.world.obstacle_edges:
      # the path intersects with an obstacle edge, so it is not a valid path
      if lines_intersect(edge, (x1.coord, x2.coord)):
        return False
    return True


  def build_rrt_tree(self, start, goal):
    self.rrt_tree.append(start)
    begin = False
    while not begin:
      x_rand = self.sample(goal)
      print(x_rand.coord)
      x_new = self.steer(start, x_rand) # includes step function
      print(x_new.coord)
      if self.obstacle_free(x_rand, x_new):
        print('poop')
        self.rrt_tree.append(x_new)
        x_new.parent = start
        start.children.append(x_new)
        self.spatial_hash_add(x_new)
        begin = True

    i = 0
    while self.dist(x_new, goal) >= self.thresh:
      x_rand = self.sample(goal)
      x_nearest = self.spatial_hash_get(x_rand, start) # use spatial hash to find nearest neighbor approximately 
      x_new = self.steer(x_nearest, x_rand)
      if self.obstacle_free(x_nearest, x_new):
        self.rrt_tree.append(x_new)
        x_new.parent = x_nearest
        x_nearest.children.append(x_new)
        self.spatial_hash_add(x_new) # add new node to spatial hashmap
        i += 1

      # if i>1000:
      #   p = self.rrt_tree[-1]
      #   p.children.append(goal)
      #   goal.parent = p
      #   self.rrt_tree.append(goal)
      #   break

    return self.rrt_tree


  def run(self) -> None:
    # construct tree backwards from goal to current node
    root_node = self.x_goal
    curr_node = self.curr_pos
    self.rrt_tree = self.build_rrt_tree(root_node, curr_node)
    self.planned_path = self.extract_path(root_node)
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
    window.rowconfigure([0,1], minsize=50, weight=1)
    window.columnconfigure([0,1], minsize=50, weight=1)
    canvas = tk.Canvas(window, bg="white", height=300, width=300)
    canvas.grid(row=0, column=0)
    python_green = "#476042"
    for node in self.rrt_tree:
      canvas.create_oval(node.coord[0]*2, node.coord[1]*2, node.coord[0]*2, node.coord[1]*2, fill=python_green)
      if node.parent:
        canvas.create_line(node.parent.coord[0]*2, node.parent.coord[1]*2, node.coord[0]*2, node.coord[1]*2)
    for (x0, y0), (x1, y1) in self.world.obstacle_edges:
      canvas.create_line(x0*2, y0*2, x1*2, y1*2)

    # create buttons at the bottom to step through simulation
    lbl_value = tk.Label(master=window, text=f"Step: {step}")
    lbl_value.grid(row=1, column=0)
    clicked = tk.BooleanVar(False)
    step_btn = tk.Button(master=window, text="Next", command=lambda: clicked.set(True))
    step_btn.grid(row=1, column=1, sticky="nsew")
    window.mainloop()
    #window.wait_variable(clicked)


if __name__=='__main__':
  world = World()
  rrtx = RRTX(world=world, x_start=Node((5,5)), x_goal=Node((85,85)), gui=True)
  rrtx.run()