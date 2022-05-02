import argparse
from collections import defaultdict
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
from shapely.geometry import Point, LineString, MultiPoint, MultiPolygon
from shapely.ops import nearest_points

from farrt.PartiallyObservablePlanner import PartiallyObservablePlanner
from farrt.node import Node
from farrt.plot import plot_polygons, plot_planner
from farrt.rrtbase import RRTBase
from farrt.runner import main
from farrt.world import World
from farrt.utils import as_multipoint, as_point, multipoint_without, pt2tuple, shapely_edge

import imageio

vertex_t = tuple[float,float]
edge_t = tuple[vertex_t,vertex_t]

class RRTStar(RRTBase):

  def __init__(self, *args, **kwargs) -> None:
    super().__init__(*args, **kwargs)

    self.rrt_tree: MultiPoint = MultiPoint()
    self.rrt_vertices: set[vertex_t] = set()
    self.rrt_edges: set[edge_t] = set()
    
    self.free_points: MultiPoint = MultiPoint()

    self.built_tree = False

  def setup_planner(self) -> None:
    """
    Build the RRT tree on the first step.
    Extract a plan from the tree.
    """
    # empty out the current planned path (should already be empty)
    self.planned_path = []
    print(f'Do first rrt plan! {pt2tuple(self.x_start_pt)} -> {pt2tuple(self.x_goal_pt)}')

    # build the tree from goal to start
    final_pt,final_cost = self.build_rrt_tree(root=self.x_goal_pt, goal_pt=self.curr_pos.coord, goal_threshold=0)
    self.built_tree = True
    print(f'First plan complete. Ends at {final_pt} with cost {final_cost}')

    # extract a plan from the tree and reverse it (to go from goal to start)
    self.planned_path = self.extract_path(endpoint=final_pt,root=self.x_goal_pt,reverse=True)
    print(f'Path: {len(self.planned_path)}')

    # display the initial plan
    # if self.gui:
    #   self.render(visualize=True)

  def handle_new_obstacles(self, new_obstacles: BaseGeometry) -> None:
    if new_obstacles.is_empty: # ignore step if no new obstacles discovered
      return
    if not len(self.planned_path): # ignore step if no path planned (may be an error case)
      print('No path planned!')
      return
    # if we have a path, check if we need to replan

    # get the full remaining path including current position
    path = [self.curr_pos.coord] + [node.coord for node in self.planned_path]
    path_line = LineString(path)

    # check if the remaining path to goal intersects with the new obstacles
    if path_line.intersects(new_obstacles):
      print('Path is inconsistent with new obstacles')
      # print(path_line)
      intersections = path_line.intersection(new_obstacles)
      # print(intersections)
      # print(new_obstacles)
      self.replan(new_obstacles=new_obstacles, draw_intersections=intersections)

  def handle_deleted_obstacles(self, deleted_obstacles: BaseGeometry) -> None:
      return super().handle_deleted_obstacles(deleted_obstacles)

  def update_planner(self) -> None:
    """
    RRT* does not have any default update process after each step.
    """
    return super().update_planner()

  def replan(self, new_obstacles: MultiPolygon, **kwargs):
    """
    Replan the path from the current position to the goal.
    1. Rerun RRT* from current position to goal
    2. Extract a plan from the tree
    """
    print('Planning...')
    if self.gui:
      self.render(draw_world_obstacles=False, save_frame=True, **kwargs)

    self.planned_path = []
    final_pt,final_cost = self.build_rrt_tree(root=self.x_goal_pt, goal_pt=self.curr_pos.coord, goal_threshold=0)
    self.planned_path = self.extract_path(endpoint=final_pt,root=self.x_goal_pt,reverse=True)

    print('Planning complete')
    # if self.gui:
    #   self.render()

  def get_min_cost_point(self, nearby_pts: MultiPoint, x_nearest: Point, x_new: Point) -> Point:
    """
    Returns the point in nearby_pts that has the minimum cost to get to x_new
    defaults to x_nearest
    """
    min_point = x_nearest
    min_cost = self.get_cost_to_goal(x_nearest) + self.get_edge_cost(x_nearest, x_new)
    for point in nearby_pts.geoms:
      if self.edge_obstacle_free(point, x_new):
        temp_cost = self.get_cost_to_goal(point) + self.get_edge_cost(point, x_new)
        if temp_cost < min_cost:
          min_point = point
          min_cost = temp_cost
    return min_point,min_cost

  def add_start_vertex(self, root: Point, /) -> None:
    """
    Add a root vertex to the tree (no parents, 0 cost)
    """
    self.rrt_tree = self.rrt_tree.union(root)

    vtx = pt2tuple(root)

    self.rrt_vertices.add(vtx)
    
    self.set_cost_to_goal(root, 0)

  def add_vertex(self, /, pt: Point, parent: Point, cost: float) -> None:
    """
    Add a vertex to the tree, add edge from parent to new vertex, set cost to reach
    """
    self.rrt_tree = self.rrt_tree.union(pt)

    vtx = pt2tuple(pt)
    parent_vtx = pt2tuple(parent)

    self.rrt_vertices.add(vtx)
    self.rrt_edges.add((parent_vtx,vtx))

    self.set_child_parent(child=pt, parent=parent)
    self.set_cost_to_goal(pt, cost)

  def reassign_parent(self, /,*, pt: Point, parent: Point, cost: float, allow_same_parent:bool = False) -> None:
    """
    Reassign the parent of a vertex to a new parent, update cost to reach
    Remove old edges from the previous parent if present
    """
    prev_parent = self.get_parent(pt)

    if prev_parent == parent:
      if allow_same_parent:
        return
      else:
        raise ValueError(f'Cannot reassign parent to same parent - child:{pt} -> prev:{prev_parent} == new:{parent}')

    vtx = pt2tuple(pt)
    old_parent_vtx = pt2tuple(prev_parent)
    new_parent_vtx = pt2tuple(parent)

    self.rrt_edges.discard((old_parent_vtx,vtx))
    self.rrt_edges.discard((vtx,old_parent_vtx))
    self.rrt_edges.add((new_parent_vtx,vtx))
    
    self.set_child_parent(child=pt, parent=parent)
    if vtx in self.get_children(prev_parent):
      print(f'Failed to remove child {pt} from parent {prev_parent} - {self.get_children(prev_parent)}')
      print(f'Sever failure: {self.parent_to_children_map[pt2tuple(prev_parent)]}')
      print(prev_parent == parent)
    assert vtx not in self.get_children(prev_parent)
    self.set_cost_to_goal(pt, cost)

  def do_rrtstar_rewiring(self, nearby_pts: MultiPoint, x_min: Point, x_new: Point) -> None:
    """
    Rewire the edges of the tree to connect x_new first if it is closer than the current parent
    Discards edges from points in nearby_pts if they can be improved by going thorugh x_new
      Then adds an edge from x_new to the point in nearby_pts (and update parent)
    """
    for x_nearby in multipoint_without(nearby_pts, x_min).geoms:
      if self.edge_obstacle_free(x_new, x_nearby):
        cost_with_new = self.get_cost_to_goal(x_new) + self.get_edge_cost(x_new, x_nearby)
        if self.get_cost_to_goal(x_nearby) > cost_with_new:
          # allow reassigning to same parent if new node is current position (b/c curr pos may be sampled many times)
          self.reassign_parent(pt=x_nearby, parent=x_new, cost=cost_with_new, allow_same_parent=x_new == self.curr_pos.coord)

  def build_rrt_tree(self, *, root: Point, goal_pt: Point, goal_threshold:float = None) -> None:
    """
    Builds the rrt tree from the root to the goal
    """
    if goal_threshold is None:
      goal_threshold = self.goal_reached_thresh
    
    # empty out the tree and vertices
    self.rrt_tree = MultiPoint()
    self.rrt_vertices.clear()
    self.rrt_edges.clear()
    # add root point to tree
    self.add_start_vertex(root)

    final_pt = None
    final_pt_cost = float('inf')

    # iterate until max iterations is reached or goal is reached
    i = 0
    while i < self.iters or final_pt is None:
      if self.display_every_n >= 1 and i % (self.display_every_n*2) == 0:
        print(f"RRT building iteration {i}")
        # if self.gui and i > 1000 and i % 1000 == 0:
        #   self.render(visualize=True)

      # sample a node, find the nearest existing node, and steer from nearest to sampled
      x_rand = self.sample_free(goal_pt, buffer_radius=0 if i > self.iters/2 and final_pt is None else self.obstacle_avoidance_radius)
      x_nearest = self.find_nearest(x_rand, pt_source=self.rrt_tree)
      x_new = self.steer(x_nearest, x_rand)

      # if there is an obstacle free path from the nearest node to the new node, analyze neighbors and add to tree
      if self.edge_obstacle_free(x_nearest, x_new):
        # find nearby points to the new point
        nearby_points = self.find_nearby_pts(x_new, radius=self.find_ball_radius(num_vertices=len(self.rrt_vertices)), pt_source=self.rrt_tree)

        # get the minimum point from the set
        x_min,min_cost = self.get_min_cost_point(nearby_points, x_nearest, x_new)

        # add the new point to the tree
        self.add_vertex(pt=x_new,parent=x_min,cost=min_cost)

        # Main difference between RRT and RRT*, modify the points in the nearest set to optimise local path costs.
        self.do_rrtstar_rewiring(nearby_points, x_min, x_new)

        # check if we've reached the goal of the tree building
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

  def set_child_parent(self, /,*, child: Point, parent: Point) -> None:
    """
    Sets the parent of a child node
    Breaks any existing relationships between child and a previous parent if present
    """
    # convert child to hashable type
    child = pt2tuple(child)

    # check if the child already has a parent - sever the connection
    old_parent = self.get_parent(child, allow_none=True)
    if old_parent is not None:
      self.get_children(old_parent).discard(child)

    # if the new parent is None, remove the child from the children map
    if parent is None and child in self.parent_to_children_map:
      del self.child_to_parent_map[child]
      return
    
    # convert parent to hashable type
    parent = pt2tuple(parent)
    # set the new parent and add the child to the children map
    self.child_to_parent_map[child] = parent
    self.parent_to_children_map[parent].add(child)

  def get_edge_cost(self, point1: Point, point2: Point) -> float:
    return point1.distance(point2)

  def get_render_kwargs(self) -> dict:
    return {
      'rrt_tree': self.rrt_tree,
      'rrt_parents': self.child_to_parent_map
    }


if __name__=='__main__':
  main(RRTStar)