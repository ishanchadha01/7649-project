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

class RRT(PartiallyObservablePlanner):

  def __init__(self, *args, **kwargs) -> None:
    super().__init__(*args, **kwargs)
    self.iters = kwargs.get('iters', 1000)
    self.eps = kwargs.get('eps', .01)

    self.max_step_length = kwargs.get('max_step_length', self.vision_radius / 2)

    self.rrt_tree: list[Node] = [] # tree to calculate path to goal
    self.goal_reached_thresh = 1


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
      return super().handle_deleted_obstacles(deleted_obstacles)

  def plan(self, **kwargs):
    print('Planning...')
    self.render(draw_world_obstacles=False, save_frame=True, **kwargs)
    self.build_rrt_tree(self.x_goal, self.curr_pos)
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
    for node2 in self.rrt_tree:
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

  def build_rrt_tree(self, start: Node, goal: Node) -> list[Node]:
    self.rrt_tree.clear()
    self.rrt_tree.append(start)

    i = 0
    while i < self.iters:
      x_rand = self.sample_free(goal)
      x_nearest = self.nearest(x_rand, start) # use spatial hash to find nearest neighbor approximately 
      x_new = self.steer(x_nearest, x_rand)
      if self.obstacle_free(x_nearest, x_new):
        self.rrt_tree.append(x_new)
        x_new.parent = x_nearest
        x_nearest.children.append(x_new)
        i += 1
        if x_new.dist(goal) <= self.goal_reached_thresh:
          break
    return self.rrt_tree

  def extract_path(self, goal: Node) -> list[Node]:
    curr = self.rrt_tree[-1] # start node / curr pos
    path = [curr]
    while not curr.same_as(goal):
      path.append(curr.parent)
      curr = curr.parent
    return list(path)

  def get_render_kwargs(self) -> dict:
    return {
      'rrt_tree': self.rrt_tree,
    }


if __name__=='__main__':
  world = World()
  rrt = RRT(world=world, x_start=Node(world.random_position(not_blocked=True)), x_goal=Node(world.random_position(not_blocked=True)), gui=True)
  rrt.run()