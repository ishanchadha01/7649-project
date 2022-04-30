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
from pyproj import Geod

from farrt.PartiallyObservablePlanner import PartiallyObservablePlanner
from farrt.node import Node
from farrt.plot import plot_polygons, plot_planner
from farrt.world import World

import imageio

class RRTX(PartiallyObservablePlanner):

  def __init__(self, *args, **kwargs) -> None:
    super().__init__(*args, **kwargs)
    self.iters = kwargs.get('iters', 1000)
    self.eps = kwargs.get('eps', .01)

    self.intersecting_edges = set()

    self.rrtx_tree: list[Node] = [] # tree to calculate path to goal
    self.goal_reached_thresh = 1
    self.pq = PriorityQueue()
    self.n = 1000
    self.neighbor_radius = math.floor((math.prod(self.world.dims) * math.log(self.n) / self.n) ** (1/2)) # radius of ball for neighbors


  def update_plan(self) -> None:
    self.rewire_neighbors()
    self.reduce_inconsistency()

  def trajectory_dist(self):
    geod = Geod(ellps="WGS84")
    traj = LineString(self.planned_path)
    return geod.geometry_length(traj)

  def rewire_neighbors(self) -> None:
    # rewires in neighbors if doing so results in better cost-to-goal
    self.cull_neighbors()
    for neighbor in self.curr_pos.in_neighbors:
      if self.curr_pos.lmc > self.curr_pos.dist(neighbor.coord) + neighbor.lmc:
        pass


  def cull_neighbors(self) -> None:
    # update running set of neighbors for curr pos
    pass

  def handle_new_obstacles(self, new_obstacles: BaseGeometry) -> None:
    if new_obstacles.is_empty:
      return
    path = [self.curr_pos.coord] + [node.coord for node in self.planned_path]
    if len(path) <= 1 or LineString(path).intersects(new_obstacles):
      print('Path is inconsistent with new obstacles')
      intersections = None
      if len(path) > 1:
        # print(LineString(path))
        intersections = LineString(path).intersection(new_obstacles)
        # print(intersections)
        # print(new_obstacles)
      self.plan(draw_intersections=intersections)

  def handle_deleted_obstacles(self, deleted_obstacles: BaseGeometry) -> None:
    # self.propagate_changes()
    # self.pq.put()
    return super().handle_deleted_obstacles(deleted_obstacles)

  def plan(self, **kwargs):
    print('Planning...')
    self.render(draw_world_obstacles=False, save_frame=True, **kwargs)
    self.build_rrtx_tree(self.x_goal, self.curr_pos)
    self.planned_path = self.extract_path(self.x_goal)
    print('Planning complete')
    # self.render()

  def sample_free(self, goal_node: Node) -> Node:
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
    for node2 in self.rrtx_tree:
      if node.dist(node2) < min_dist:
        min_dist = node.dist(node2)
        min_node = node2
    return min_node

  def steer(self, x_nearest: Node, x_rand: Node) -> Node:
    dist = x_nearest.dist(x_rand)
    if dist == 0:
      return x_rand
    factor = min(1, self.max_step_length / dist)
    newCoord = x_nearest.edgeTo(x_rand).interpolate(factor, normalized=True)
    return Node(coord=newCoord)

  def obstacle_free(self, x_nearest: Node, x_new: Node) -> bool:
    return not self.detected_obstacles.intersects(x_nearest.edgeTo(x_new))


  def build_rrtx_tree(self, start: Node, goal: Node) -> list[Node]:
    self.rrt_tree.clear()
    self.rrt_tree.append(start)
    x_new = start
    i = 0
    while i < self.iters:
      x_rand = self.sample_free(goal)
      x_nearest = self.nearest(x_rand, start) # use spatial hash to find nearest neighbor approximately
      x_new = self.steer(x_nearest, x_rand)
      if self.obstacle_free(x_nearest, x_new):
        self.rrtx_tree.append(x_new)
        x_new.parent = x_nearest
        x_nearest.children.append(x_new)
        i += 1
        if x_new.dist(goal) <= self.goal_reached_thresh:
          break
    return self.rrtx_tree

  def extract_path(self, goal: Node) -> list[Node]:
    curr = self.rrtx_tree[-1] # start node / curr pos
    path = [curr]
    while not curr.same_as(goal):
      path.append(curr.parent)
      curr = curr.parent
    return list(path)

  def get_render_kwargs(self) -> dict:
    return {
      'rrt_tree': self.rrtx_tree,
    }


if __name__=='__main__':
  world = World()
  rrtx = RRTX(world=world, x_start=Node(world.random_position(not_blocked=True)), x_goal=Node(world.random_position(not_blocked=True)), gui=True)
  rrtx.run()
