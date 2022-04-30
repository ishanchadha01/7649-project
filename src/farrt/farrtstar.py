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
from shapely.geometry import Point, LineString, MultiPoint
from shapely.ops import nearest_points

from farrt.PartiallyObservablePlanner import PartiallyObservablePlanner
from farrt.node import Node
from farrt.plot import plot_polygons, plot_planner
from farrt.world import World
from farrt.utils import as_multipoint, multipoint_without, shapely_edge

import imageio

vertex_t = tuple[float,float]
edge_t = tuple[vertex_t,vertex_t]

class FARRT(PartiallyObservablePlanner):

  def __init__(self, *args, **kwargs) -> None:
    super().__init__(*args, **kwargs)
    self.iters = kwargs.get('iters', 2000)
    self.eps = kwargs.get('eps', .01)

    self.detected_obstacles = BaseGeometry()
    self.steer_distance = kwargs.get('max_step_length', self.vision_radius / 3)

    self.obstacle_avoidance_radius = kwargs.get('obstacle_avoidance_radius', self.steer_distance * 2/3)

    self.x_start_pt = self.x_start.coord
    self.x_goal_pt = self.x_goal.coord

    self.farrt_tree = MultiPoint()
    self.farrt_vertices: set[vertex_t] = set()
    self.farrt_edges: set[edge_t] = set()
    self.parent_map: dict[vertex_t, vertex_t] = {}
    self.cost_to_reach: dict[vertex_t, float] = {}

    self.goal_reached_thresh = 1
    self.built_tree = False


  def handle_new_obstacles(self, new_obstacles: BaseGeometry) -> None:
    if not self.built_tree:
      self.do_first_plan()
      return
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
      self.replan(draw_intersections=intersections)

  def handle_deleted_obstacles(self, deleted_obstacles: BaseGeometry) -> None:
      return super().handle_deleted_obstacles(deleted_obstacles)

  def update_plan(self) -> None:
      return super().update_plan()

  def do_first_plan(self) -> None:
    print(f'Do first farrt plan! {self.x_start_pt.coords[0]} -> {self.x_goal_pt.coords[0]}')
    self.planned_path = []
    final_pt,final_cost = self.build_farrt_tree(root=self.x_goal_pt, goal_pt=self.curr_pos.coord, goal_threshold=0)
    self.built_tree = True
    print(f'First plan complete. Ends at {final_pt} with cost {final_cost}')
    self.planned_path = self.extract_path(endpoint=final_pt,root=self.x_goal_pt)
    print(f'Path: {len(self.planned_path)}')
    self.render(visualize=True)

  def replan(self, **kwargs):
    print('Planning...')
    if self.gui:
      self.render(draw_world_obstacles=False, save_frame=True, **kwargs)

    self.planned_path = []
    final_pt,final_cost = self.build_farrt_tree(root=self.x_goal_pt, goal_pt=self.curr_pos.coord, goal_threshold=0)
    self.planned_path = self.extract_path(endpoint=final_pt,root=self.x_goal_pt)

    print('Planning complete')
    # if self.gui:
    #   self.render()

  def sample_free(self, goal_pt: Point, buffer_radius:float = None) -> Point:
    if random.random() < self.eps: # some % chance of returning goal node
      return goal_pt
    if buffer_radius is None:
      buffer_radius = self.obstacle_avoidance_radius
    rand_pos = self.world.random_position()
    while self.detected_obstacles.intersects(rand_pos.buffer(buffer_radius)):
      rand_pos = self.world.random_position()
    return rand_pos

  def find_nearest(self, x_rand: Point) -> Point:
    nearest_geoms = nearest_points(multipoint_without(self.farrt_tree, x_rand), x_rand)
    nearest_pt = nearest_geoms[0]
    return nearest_pt

  def steer(self, x_nearest: Point, x_rand: Point) -> Point:
    dist = x_nearest.distance(x_rand)
    if dist == 0:
      return x_rand
    factor = min(1, self.steer_distance / dist)
    newCoord = shapely_edge(x_nearest,x_rand).interpolate(factor, normalized=True)
    return newCoord

  def edge_obstacle_free(self, x_nearest: Point, x_new: Point) -> bool:
    return not self.detected_obstacles.intersects(shapely_edge(x_nearest,x_new))

  def find_nearby_pts(self, x_new: Point) -> MultiPoint:

    def find_ball_radius():
      unit_volume = math.pi
      num_vertices = len(self.farrt_vertices)
      dimensions = len(self.world.dims)
      minx,miny,maxx,maxy = self.world.getBounds()
      gamma = (2**dimensions)*(1.0 + 1.0/dimensions) * (maxx - minx) * (maxy - miny)
      ball_radius = min(
        ((gamma/unit_volume) * math.log(num_vertices) / num_vertices)**(1.0/dimensions),
        self.steer_distance
      )
      return ball_radius
    
    nearby_points = multipoint_without(self.farrt_tree, x_new).intersection(x_new.buffer(find_ball_radius()))
    return as_multipoint(nearby_points)

  def get_min_cost_point(self, nearby_pts: MultiPoint, x_nearest: Point, x_new: Point) -> Point:
    """
    Returns the point in nearby_pts that has the minimum cost to get to x_new
    defaults to x_nearest
    """
    min_point = x_nearest
    min_cost = self.get_cost_to_reach(x_nearest) + self.get_edge_cost(x_nearest, x_new)
    for point in nearby_pts.geoms:
      if self.edge_obstacle_free(point, x_new):
        temp_cost = self.get_cost_to_reach(point) + self.get_edge_cost(point, x_new)
        if temp_cost < min_cost:
          min_point = point
          min_cost = temp_cost
    return min_point,min_cost

  def add_start_vertex(self, x_start: Point) -> None:
    self.farrt_tree = self.farrt_tree.union(x_start)

    vtx = self.as_vertex(x_start)

    self.farrt_vertices.add(vtx)
    
    self.set_cost_to_reach(x_start, 0)

  def add_vertex(self, pt: Point, parent: Point, cost: float) -> None:
    self.farrt_tree = self.farrt_tree.union(pt)

    vtx = self.as_vertex(pt)
    parent_vtx = self.as_vertex(parent)

    self.farrt_vertices.add(vtx)
    self.farrt_edges.add((parent_vtx,vtx))

    self.set_parent(pt, parent)
    self.set_cost_to_reach(pt, cost)

  def reassign_parent(self, pt: Point, parent: Point, cost: float) -> None:
    prev_parent = self.get_parent(pt)

    vtx = self.as_vertex(pt)
    old_parent_vtx = self.as_vertex(prev_parent)
    new_parent_vtx = self.as_vertex(parent)

    self.farrt_edges.discard((old_parent_vtx,vtx))
    self.farrt_edges.discard((vtx,old_parent_vtx))
    self.farrt_edges.add((new_parent_vtx,vtx))
    
    self.set_parent(pt, parent)
    self.set_cost_to_reach(pt, cost)

  def do_rrtstar_rewiring(self, nearby_pts: MultiPoint, x_min: Point, x_new: Point) -> None:
    """
    Rewire the edges of the tree to connect x_new first if it is closer than the current parent
    Discards edges from points in nearby_pts if they can be improved by going thorugh x_new
      Then adds an edge from x_new to the point in nearby_pts (and update parent)
    """
    for x_nearby in multipoint_without(nearby_pts, x_min).geoms:
      if self.edge_obstacle_free(x_nearby, x_new):
        cost_with_new = self.get_cost_to_reach(x_new) + self.get_edge_cost(x_nearby, x_new)
        if self.get_cost_to_reach(x_nearby) > cost_with_new:
            self.reassign_parent(pt=x_nearby, parent=x_new, cost=cost_with_new)

  def reached_goal(self, x_new: Point, *, goal: Point = None, threshold:float = None) -> bool:
    if goal is None:
      goal = self.x_goal_pt
    if threshold is None:
      threshold = self.goal_reached_thresh
    return x_new.distance(goal) < self.goal_reached_thresh

  def build_farrt_tree(self, *, root: Point, goal_pt: Point, goal_threshold:float = None) -> None:
    """
    Builds the farrt tree from the root to the goal
    """
    if goal_threshold is None:
      goal_threshold = self.goal_reached_thresh
    self.farrt_tree = MultiPoint()
    self.farrt_vertices.clear()
    self.farrt_edges.clear()
    self.add_start_vertex(root)#self.x_start_pt

    final_pt = None
    final_pt_cost = float('inf')

    i = 0
    while i < self.iters or final_pt is None:
      if self.display_every_n >= 1 and i % (self.display_every_n*2) == 0:
        print(f"RRT building iteration {i}")
        if i > 1000 and i % 1000 == 0:
          self.render(visualize=True)

      x_rand = self.sample_free(goal_pt, buffer_radius=self.obstacle_avoidance_radius if i < self.iters/2 else 0)
      x_nearest = self.find_nearest(x_rand)
      x_new = self.steer(x_nearest, x_rand)
      if self.edge_obstacle_free(x_nearest, x_new):
        # find nearby points to the new point
        nearby_points = self.find_nearby_pts(x_new)

        # get the minimum point from the set
        x_min,min_cost = self.get_min_cost_point(nearby_points, x_nearest, x_new)

        # add the new point to the tree
        self.add_vertex(pt=x_new,parent=x_min,cost=min_cost)

        # Main difference between RRT and RRT*, modify the points in the nearest set to optimise local path costs.
        self.do_rrtstar_rewiring(nearby_points, x_min, x_new)

        if self.reached_goal(x_new, goal=goal_pt, threshold=goal_threshold):
          if self.built_tree: # subsequent runs should just terminate once goal is reached
            final_pt = x_new
            break
          else: # keep searching and update the shortest path
            if min_cost < final_pt_cost:
              final_pt = x_new
              final_pt_cost = min_cost
      i += 1
    return final_pt,final_pt_cost

  def extract_path(self, *, endpoint: Point, root: Point) -> list[Node]:
    curr = endpoint
    path: list[Point] = []
    while curr != root:
      # i = len(path)
      # if self.display_every_n >= 1 and i % self.display_every_n == 0:
      #   print(f"path point {i}: {curr.coords[0]}")
      if curr is None:
        print("ERROR: curr is None!", list(map(str,path)))
        self.render(visualize=True)
        break
      curr = self.get_parent(curr)
      path.append(curr)
    # path.append(self.curr_pos.coord)
    # path.reverse() # curr pos first
    node_path = []
    for i,pt in enumerate(path):
      parent = self.curr_pos if i == 0 else node_path[i-1]
      node_path.append(Node(pt,parent))
    return node_path

  def as_vertex(self, pt: Point) -> vertex_t:
    return pt.coords[0]

  def get_parent(self, point: Point) -> Point:
    return Point(self.parent_map[point.coords[0]])

  def set_parent(self, point: Point, parent: Point) -> None:
    self.parent_map[point.coords[0]] = parent.coords[0]

  def get_cost_to_reach(self, point: Point) -> float:
    return self.cost_to_reach[point.coords[0]]

  def set_cost_to_reach(self, point: Point, cost: float) -> None:
    self.cost_to_reach[point.coords[0]] = cost

  def get_edge_cost(self, point1: Point, point2: Point) -> float:
    return point1.distance(point2)

  def get_render_kwargs(self) -> dict:
    return {
      'rrt_tree': self.farrt_tree,
      'rrt_parents': self.parent_map
    }


if __name__=='__main__':
  # extract run args run_count
  parser = argparse.ArgumentParser()
  parser.add_argument('-rc', type=int, dest='run_count')
  args = parser.parse_args()
  filtered_args = {k: v for k, v in vars(args).items() if v is not None}

  world = World()
  farrt = FARRT(world=world, x_start=Node(world.random_position(not_blocked=True)), x_goal=Node(world.random_position(not_blocked=True)), gui=True, **filtered_args)
  farrt.run()