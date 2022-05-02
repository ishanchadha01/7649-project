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
from farrt.world import World
from farrt.utils import as_multipoint, as_point, multipoint_without, pt2tuple, shapely_edge

import imageio

vertex_t = tuple[float,float]
edge_t = tuple[vertex_t,vertex_t]

class RRTBase(PartiallyObservablePlanner):

  def __init__(self, *args, **kwargs) -> None:
    self.iters = kwargs.pop('iters', 2000)
    self.eps = kwargs.pop('eps', .01)

    self.steer_distance = kwargs.pop('max_step_length', None)
    self.obstacle_avoidance_radius = kwargs.pop('obstacle_avoidance_radius', None)
    self.goal_reached_thresh = kwargs.pop('goal_reached_thresh', None)
    super().__init__(*args, **kwargs)

    if self.steer_distance is None:
      self.steer_distance = self.vision_radius / 3
    if self.obstacle_avoidance_radius is None:
      self.obstacle_avoidance_radius = self.steer_distance * 2/3
    if self.goal_reached_thresh is None:
      self.goal_reached_thresh = 1

    # get shapely Point version of start/goal
    self.x_start_pt = self.x_start.coord
    self.x_goal_pt = self.x_goal.coord
    
    self.child_to_parent_map: dict[vertex_t, vertex_t] = {}
    self.parent_to_children_map: defaultdict[vertex_t, set[vertex_t]] = defaultdict(set)

    self.cost_to_goal: dict[vertex_t, float] = {}

  def find_ball_radius(self, num_vertices: int) -> float:
    """
    Determine the radius of the ball to search for nearby points
    """
    unit_volume = math.pi
    num_vertices += 1 # not sure this is necessary but without it the default value is always 0??
    dimensions = len(self.world.dims)
    minx,miny,maxx,maxy = self.world.getBounds()
    free_space = self.world.getBoundingPolygon().area - self.detected_obstacles.area
    gamma = (2**dimensions)*(1.0 + 1.0/dimensions) * free_space
    ball_radius = min(
      ((gamma/unit_volume) * math.log(num_vertices) / num_vertices)**(1.0/dimensions),
      self.steer_distance
    )
    return ball_radius
  
  def sample_free(self, goal_pt: Point, *, buffer_radius:float = None, require_free:bool = True) -> Point:
    """
    Sample a free point in the world
    Return the goal with some low probability
    Ensure the new point is further than the buffer radius from existing obstacles
    """
    if random.random() < self.eps and goal_pt is not None: # some % chance of returning goal node
      return as_point(goal_pt)
    
    # use default buffer for obstacle avoidance
    if buffer_radius is None:
      buffer_radius = self.obstacle_avoidance_radius
    
    # sample a random point from the world
    rand_pos = self.world.random_position()

    # exit early if we don't need to smaple free
    if not require_free:
      return rand_pos

    # ensure that sampled point is not too close to detected obstacles
    while self.detected_obstacles.intersects(rand_pos.buffer(buffer_radius)):
      rand_pos = self.world.random_position()
    return rand_pos

  def find_nearest(self, x_rand: Point, /,*, pt_source: MultiPoint) -> Point:
    """
    Find the nearest point in the tree to the random point
    """
    nearest_geoms = nearest_points(multipoint_without(pt_source, x_rand), x_rand)
    nearest_pt = nearest_geoms[0]
    return nearest_pt

  def steer(self, x_nearest: Point, x_rand: Point) -> Point:
    """
    Steer from the nearest point to the random point
    Limit the step to the steer distance of the planner
    """
    dist = x_nearest.distance(x_rand)
    if dist == 0: # if the points are the same, just return the sampled point
      return x_rand
    
    # factor is max value = 1 (allow steps shorter than steer_distance if sampled point is very close to nearest)
    factor = min(1, self.steer_distance / dist)

    # interpolate along the line between the nearest and sampled points
    newCoord = shapely_edge(x_nearest,x_rand).interpolate(factor, normalized=True)
    return newCoord

  def point_obstacle_free(self, v_new: Point) -> bool:
    """
    Check if the point is obstacle free
    """
    return not self.detected_obstacles.contains(as_point(v_new))

  def edge_obstacle_free(self, x_nearest: Point, x_new: Point) -> bool:
    """
    Check if the edge between the nearest and new points is free of obstacles
    """
    return not self.detected_obstacles.intersects(shapely_edge(x_nearest,x_new))

  def find_nearby_pts(self, x_new: Point, /,*, radius:float, pt_source: MultiPoint) -> MultiPoint:
    """
    Find all points in the tree within some radius of the new point
    Used for finding shortest paths and rewiring
    """
    # print(f'Finding nearby points within {radius} of {x_new}')
    # print(f'Searching {len(pt_source.geoms)} points in source within dist={x_new.distance(pt_source)} of {x_new} - radius={radius}')
    # print(f'  {pt_source}')
    # print(f' dist: {x_new.distance(pt_source)}')
    nearby_points = as_multipoint(x_new.buffer(radius, 128).intersection(multipoint_without(pt_source, x_new)))
    # print(f'  Found {len(nearby_points.geoms)} nearby points{" out of {}".format(pt_source) if len(nearby_points.geoms) == 0 else ""}')
    return nearby_points

  def reached_goal(self, x_new: Point, *, goal: Point = None, threshold:float = None) -> bool:
    """
    Check if the new point is close enough to the goal to be considered reached
    """
    if goal is None: # default to the actual goal of the planner
      goal = self.x_goal_pt
    if threshold is None:
      threshold = self.goal_reached_thresh
    
    # check that the distance is below the threshold
    return x_new.distance(goal) < self.goal_reached_thresh

  def extract_path(self, *, endpoint: Point, root: Point, reverse:bool = True) -> list[Node]:
    """
    Extracts the path from the root of rrt tree to the endpoint
    Done by starting from endpoint in tree and iterating over parents until root is reached
    """
    curr = endpoint
    path: list[Point] = []
    while curr != root:
      if curr is None:
        print("ERROR: curr is None!", list(map(str,path)))
        self.render(visualize=True)
        break
      if reverse: # get parent before adding (such that final path will include root but not endpoint)
        curr = self.get_parent(curr)
        path.append(curr)
      else: # add point before getting parent (such that final path will include endpoint but not root)
        path.append(curr)
        curr = self.get_parent(curr)
      if path[-1] in path[:-1]:
        print("ERROR: path contains a node more than once!")
        self.render(visualize=True, extra_points=path)
        break
    if not reverse: # invert condition because path is already reversed since iterating backwards via parents
      path.reverse() # curr pos first
    
    # convert to nodes with parent relationships
    node_path = []
    for i,pt in enumerate(path):
      parent = self.curr_pos if i == 0 else node_path[i-1]
      node_path.append(Node(pt,parent))
    return node_path

  def get_parent(self, point: Point, /,*, allow_none:bool = False) -> Point:
    if pt2tuple(point) not in self.child_to_parent_map: # happens during severing for farrtstar
      if allow_none:
        return None
      else:
        print("ERROR: parent is None!", point)
        self.render(visualize=True,save_frame=True,extra_points=[point])
        raise ValueError(f"No parent found for point {point}")
    parent = self.child_to_parent_map[pt2tuple(point)]
    return Point(parent)

  def get_children(self, point: Point, /) -> set[vertex_t]:
    return self.parent_to_children_map[pt2tuple(point)]

  def get_cost_to_goal(self, point: Point) -> float:
    return self.cost_to_goal[pt2tuple(point)]

  def set_cost_to_goal(self, point: Point, cost: float) -> None:
    self.cost_to_goal[pt2tuple(point)] = cost

  def get_edge_cost(self, point1: Point, point2: Point) -> float:
    return as_point(point1).distance(as_point(point2))
