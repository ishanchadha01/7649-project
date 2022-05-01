import argparse
from collections import defaultdict
import os
from copy import deepcopy
import random
import math
import shutil
import tkinter as tk
import numpy as np
from farrt.rrtstar import RRTStar
from scipy.spatial import KDTree
from queue import PriorityQueue
from matplotlib import pyplot as plt

from shapely.geometry.base import BaseGeometry
from shapely.geometry import Point, LineString, MultiPoint, MultiPolygon
from shapely.ops import nearest_points

from farrt.PartiallyObservablePlanner import PartiallyObservablePlanner
from farrt.rrtstar import RRTStar
from farrt.node import Node
from farrt.plot import plot_polygons, plot_planner
from farrt.world import World
from farrt.utils import as_point, as_multipoint, multipoint_without, pt2tuple, shapely_edge

import imageio

vertex_t = tuple[float,float]
edge_t = tuple[vertex_t,vertex_t]

class RRTX(RRTStar):

  def __init__(self, *args, **kwargs) -> None:
    super().__init__(*args, **kwargs)
    self.iters = kwargs.get('iters', 1000)
    self.eps = kwargs.get('eps', .01)

    self.intersecting_edges = set()
    self.lookahead_estimate: dict[vertex_t, float] = defaultdict(float('inf'))

    self.rrtx_tree: MultiPoint = MultiPoint()
    self.goal_reached_thresh = 1
    self.pq: PriorityQueue()
    self.inconsistent_set: set(vertex_t) #basically priority queue but in set form
    self.orphans = set(vertex_t)

    self.n = 1000
    self.neighbor_radius = math.floor((math.prod(self.world.dims) * math.log(self.n) / self.n) ** (1/2)) # radius of ball for neighbors


  def update_plan(self) -> None:
    if self.lookahead_estimate[self.curr_pos] != self.cost_to_reach[self.curr_pos]:
      nearby_pts = self.find_nearest(self.curr_pos)
      x_nearest = self.find_nearby_pts(self.curr_pos)
      x_min, _ = self.get_min_cost_point(nearby_pts, x_nearest, self.curr_pos)
      self.do_rrtx_rewiring(nearby_pts, x_min, self.curr_pos)
      self.update_key(self.curr_pos)
      for neighbor in self.children_map[self.curr_pos]:
        if self.lookahead_estimate[self.curr_pos] != self.cost_to_reach[self.curr_pos]:
          self.update_key(neighbor)
    self.reduce_inconsistency()

  def put_pq(self, key, node):
    self.pq.put((key, pt2tuple(node)))

  def update_key(self, node: Point) -> None:
    key = min(self.get_lmc(node), self.get_cost_to_reach(node))
    self.inconsistent_set.add(pt2tuple(node))
    self.put_pq(key, node)

  def reduce_inconsistency(self) -> None:
    while self.pq.qsize() > 0\
      and (self.get_lmc(self.curr_pos) != self.get_cost_to_reach(self.curr_pos)\
      or self.get_cost_to_reach(self.curr_pos) == float('inf')\
      or pt2tuple(self.curr_pos) in self.inconsistent_set):

      node = as_point(self.pq.pop())
      if self.get_lmc(node) != self.get_cost_to_reach(node):
        self.update_lookahead(node)
        nearby_pts = self.find_nearest(self.curr_pos)
        x_nearest = self.find_nearby_pts(self.curr_pos)
        x_min, _ = self.get_min_cost_point(nearby_pts, x_nearest, self.curr_pos)
        self.do_rrtx_rewiring(nearby_pts, x_min, node)
      self.set_cost_to_reach(node, self.get_lmc(node))

  def get_lmc(self, node: Point):
    return self.lookahead_estimate[pt2tuple(node)]
  
  def set_lmc(self, node: Point, lmc: float):
    self.lookahead_estimate[pt2tuple(node)] = lmc

  def update_lookahead(self, node):
    nearby_pts = self.find_nearest(self.curr_pos)
    new_parent = self.parent_map[node]
    for x_nearby in nearby_pts:
      if self.get_lmc(node) > self.get_lmc(x_nearby) + self.get_edge_cost(x_nearby, node):
        new_parent = x_nearby
    self.set_child_parent(child=node, parent=new_parent)

  def do_rrtx_rewiring(self, nearby_pts: MultiPoint, x_min: Point, x_new: Point) -> None:
    """
    Similar to RRT* rewire except using lookahead estimate instead of cost to goal
    """
    for x_nearby in multipoint_without(nearby_pts, x_min).geoms:
      if self.edge_obstacle_free(x_nearby, x_new):
        cost_with_new = self.get_lmc(x_new) + self.get_edge_cost(x_nearby, x_new)
        if self.get_lmc(x_nearby) > cost_with_new:
            self.set_lmc(cost_with_new)
            self.reassign_parent(pt=x_nearby, parent=x_new, cost=cost_with_new)
            if self.get_cost_to_reach(x_nearby) != self.get_lmc(x_nearby):
              self.update_key(x_nearby)

  def replan(self, new_obstacles: MultiPolygon, **kwargs):
    conflicting_pts = as_multipoint(self.rrt_tree.intersection(self.detected_obstacles))
    # get rid of points in obstacles
    # remove edges to their neighbors
    # if their neighbors arent in obstacle, add to orphan set
    # propagate this change to all descendants of these orphan nodes
    # for every orphan and all of its descendants, set cost to reach to inf
    # for every orphan and all of its descendants, add to PQ/inconsistent set, with key being their previous LMC + half the perimeter of the obstacle
    # since the update to the lookahead estimate is at least that much]
    # sample from the orphan set based on lookahead values, prolly bidirectional search from curr pos and path among orphan nodes
    # if time, do aco every time we run into an obstacle to avoid dead ends, similar to reasoning for APF for farrt

  def handle_new_obstacles(self, new_obstacles: BaseGeometry) -> None:
    super().handle_new_obstacles(new_obstacles)


  def handle_deleted_obstacles(self, deleted_obstacles: BaseGeometry) -> None:
    return super().handle_deleted_obstacles(deleted_obstacles)

    
  def get_render_kwargs(self) -> dict:
    return {
      'rrt_tree': self.rrtx_tree,
    }


if __name__=='__main__':
  world = World()
  rrtx = RRTX(world=world, x_start=Node(world.random_position(not_blocked=True)), x_goal=Node(world.random_position(not_blocked=True)), gui=True)
  rrtx.run()
