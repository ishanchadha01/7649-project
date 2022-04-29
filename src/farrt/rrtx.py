import argparse
import os
from copy import deepcopy
import random
import math
import shutil
import tkinter as tk
import numpy as np
from scipy.spatial import KDTree
from queue import PriorityQueue
from matplotlib import pyplot as plt

from shapely.geometry.base import BaseGeometry
from shapely.geometry import Point, LineString

from farrt.PartiallyObservablePlanner import PartiallyObservablePlanner
from farrt.node import Node
from farrt.plot import plot_polygons, plot_planner
from farrt.world import World

import imageio

class RRTX(PartiallyObservablePlanner):

  def __init__(self, run_count, *args, **kwargs) -> None:
    self.gui = kwargs.pop('gui', True)
    self.run_count = run_count
    super().__init__(*args, **kwargs)
    self.iters = kwargs.get('iters', 1000)
    self.MIN_EXPLORATION_DISTANCE = kwargs.get('MIN_EXPLORATION_DISTANCE', 0)
    self.MAX_EXPLORATION_DISTANCE = kwargs.get('MAX_EXPLORATION_DISTANCE', 10)
    self.eps = kwargs.get('eps', .01)
    self.planned_path = []

    self.detected_obstacles = BaseGeometry()
    self.vision_radius = kwargs.get('vision_radius', 10)

    self.thresh = 1
    self.rrt_tree: list[Node] = [] # tree to calculate path to goal
    self.step_size = 0.2
    self.pq = PriorityQueue()
    self.n = 1000
    self.radius = math.floor((math.prod(self.world.dims) * math.log(self.n) / self.n) ** (1/2)) # radius of ball for neighbors

    


  def observe_world(self) -> None:
    observations = self.world.make_observations(self.curr_pos, self.vision_radius)
    new_obstacles = observations - self.detected_obstacles
    if not new_obstacles.is_empty: # new obstacles detected
      # self.reduce_inconsistency()
      path = [self.curr_pos.coord] + [node.coord for node in self.planned_path]
      if len(path) <= 1 or LineString(path).intersects(new_obstacles):
        self.plan()
      pass
    deleted_obstacles = self.detected_obstacles - observations
    if not deleted_obstacles.is_empty: # obstacles disappeared
      # self.propagate_changes()
      # self.pq.put()
      pass
    self.detected_obstacles = self.detected_obstacles.union(observations)

  def plan(self):
    self.build_rrt_tree(self.x_goal, self.curr_pos)
    self.planned_path = self.extract_path(self.x_goal)

  def sample(self, goal_node: Node) -> Node:
    if random.random() < self.eps: # some % chance of returning goal node
      return goal_node
    # return uniform floating point position within the world
    rand_pos = self.world.random_position()
    while rand_pos is None or self.detected_obstacles.contains(rand_pos):
      rand_pos = self.world.random_position()
    return Node(coord=rand_pos)


  def nearest(self, node: Node, fallback: Node) -> Node:
    min_dist = float('inf')
    min_node = fallback
    for node2 in self.rrt_tree:
      if node.dist(node2) < min_dist:
        min_dist = node.dist(node2)
        min_node = node2
    return min_node


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


  def update_obstacles(self):
    if self.world.obstacle_detected():
      self.reduce_inconsistency()
    if self.world.obstacle_vanished():
      self.propagate_changes()
      self.pq.put()


  def steer(self, x_nearest: Node, x_rand: Node) -> Node:
    dist = x_nearest.dist(x_rand)
    if dist == 0:
      return x_rand
    factor = min(1, self.vision_radius * 0.5 / dist)
    newCoord = x_nearest.edgeTo(x_rand).interpolate(factor, normalized=True)
    return Node(coord=newCoord)

    # factor = self.step_size / self.dist(x1, x2)
    # coord = tuple([factor * (x2_d - x1_d) + x1_d for x1_d, x2_d in zip(x1.coord, x2.coord)])
    # return Node(coord=coord)

    # return deepcopy(x_rand)


  def obstacle_free(self, x_nearest: Node, x_new: Node) -> bool:
    return not self.detected_obstacles.intersects(x_nearest.edgeTo(x_new))


  def build_rrt_tree(self, start: Node, goal: Node) -> list[Node]:
    self.rrt_tree.clear()
    self.rrt_tree.append(start)
<<<<<<< HEAD
    x_new = start
=======
    # begin = False
    # while not begin:
    #   x_rand = self.sample(goal)
    #   # print(x_rand.coord)
    #   x_new = self.steer(start, x_rand) # includes step function
    #   # print(x_new.coord)
    #   if self.obstacle_free(x_rand, x_new):
    #     # print('poop')
    #     self.rrt_tree.append(x_new)
    #     x_new.parent = start
    #     start.children.append(x_new)
    #     begin = True

>>>>>>> 4ecb18937c8c8f87c085004c0b528e216e694559
    i = 0
    while i < self.iters:
      x_rand = self.sample(goal)
      x_nearest = self.nearest(x_rand, start) # use spatial hash to find nearest neighbor approximately 
      x_new = self.steer(x_nearest, x_rand)
      if self.obstacle_free(x_nearest, x_new):
        self.rrt_tree.append(x_new)
        x_new.parent = x_nearest
        x_nearest.children.append(x_new)
        i += 1
        if x_new.dist(goal) <= self.thresh:
          break

    return self.rrt_tree

  def extract_path(self, start: Node) -> list[Node]:
    curr = self.rrt_tree[-1]
    path = [curr]
    while not curr.same_as(start):
      path.append(curr.parent)
      curr = curr.parent
    return list(reversed(path))


  def run(self) -> None:
    # construct tree backwards from goal to current node
    # root_node = self.x_goal
    # curr_node = self.curr_pos
    self.observe_world()
    step = 0
<<<<<<< HEAD
    while curr_node != self.x_goal:
      next_node = self.planned_path.pop()
      self.update_obstacles()
=======
    if self.gui:
      fig,ax = (None,None)#plt.subplots()
      if os.path.exists(f'rrtx-gifs/{self.run_count}'):
        shutil.rmtree(f'rrtx-gifs/{self.run_count}')
      os.mkdir(f'rrtx-gifs/{self.run_count}')
    filenames = []
    while not self.curr_pos.same_as(self.x_goal):
      if self.gui:
        name = f'rrtx-gifs/{self.run_count}/{step}.png'
        self.render(fig,ax,step,save_file=name)
        filenames.append(name)

      next_node = self.step()
      self.curr_pos = next_node
      self.observe_world()
>>>>>>> 4ecb18937c8c8f87c085004c0b528e216e694559

      print(f'Step: {step} - Distance: {self.curr_pos.dist(self.x_goal)}')

      step += 1

    if self.gui:
      name = f'rrtx-gifs/{self.run_count}/{step}.png'
      self.render(fig,ax,step,save_file=name)
      for i in range(10):
        filenames.append(name)
      
      with imageio.get_writer(f'rrtx-gifs/run-{self.run_count}.gif', mode='I') as writer:
        for filename in filenames:
            image = imageio.imread(filename)
            writer.append_data(image)
      # for filename in set(filenames):
      #   os.remove(filename)
      shutil.rmtree(f'rrtx-gifs/{self.run_count}')

  def step(self) -> Node:
    if len(self.planned_path) == 0:
      return self.curr_pos
    return self.planned_path.pop()

<<<<<<< HEAD

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
    for (x0, y0), (x1, y1) in self.world.obstacles:
      canvas.create_line(x0*2, y0*2, x1*2, y1*2)

    # create buttons at the bottom to step through simulation
    lbl_value = tk.Label(master=window, text=f"Step: {step}")
    lbl_value.grid(row=1, column=0)
    clicked = tk.BooleanVar(value=False)
    step_btn = tk.Button(master=window, text="Next", command=lambda: clicked.set(True))
    step_btn.grid(row=1, column=1, sticky="nsew")
    window.mainloop()
    #window.wait_variable(clicked)
=======
  def render(self,fig,ax,step:int,save_file=None):
    # fig.clear()
    fig,ax = plt.subplots()
    plot_planner(fig_ax=(fig,ax), world=self.world, observations=self.detected_obstacles, curr_pos=self.curr_pos, goal=self.x_goal, rrt_tree=self.rrt_tree, planned_path=self.planned_path)
    if save_file is not None:
      plt.savefig(save_file)
    if save_file is None or step % 10 == -1:
      plt.show()
    plt.close()


>>>>>>> 4ecb18937c8c8f87c085004c0b528e216e694559


if __name__=='__main__':
  # extract run args run_count
  parser = argparse.ArgumentParser()
  parser.add_argument('-rc', type=int, required=True)
  args = parser.parse_args()
  run_count = args.rc
  
  world = World()
  rrtx = RRTX(world=world, x_start=Node((5,5)), x_goal=Node((85,85)), gui=True, run_count=run_count)
  rrtx.run()