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
from farrt.rrtstar import RRTStar
from farrt.world import World
from farrt.utils import as_multipoint, multipoint_without, pt2tuple, shapely_edge

import imageio

vertex_t = tuple[float,float]
edge_t = tuple[vertex_t,vertex_t]

class FARRTStar(RRTStar):

  def __init__(self, *args, **kwargs) -> None:
    super().__init__(*args, **kwargs)

    self.free_points: MultiPoint = MultiPoint()

  def replan(self, **kwargs):
    """
    Replan the path from the current position to the goal.
    1. Sever nodes within obstacles
    2. Apply potential field update to remaining points
    3. Rewire free points into tree
    4. Extract a plan from the tree
    """
    print('Planning...')
    if self.gui:
      self.render(draw_world_obstacles=False, save_frame=True, **kwargs)

    severed_pts, closest_parents, free_pts = self.do_tree_severing()


    print('Planning complete')
    # if self.gui:
    #   self.render()

  def do_tree_severing(self):
    conflicting_pts = as_multipoint(self.farrt_tree.intersection(self.detected_obstacles))
    closest_parents = set()
    for pt in conflicting_pts.geoms:
      closest_parents.add(self.get_parent(pt))



if __name__=='__main__':
  # extract run args run_count
  parser = argparse.ArgumentParser()
  parser.add_argument('-rc', type=int, dest='run_count')
  args = parser.parse_args()
  filtered_args = {k: v for k, v in vars(args).items() if v is not None}

  world = World()
  farrt_star = FARRTStar(world=world, x_start=Node(world.random_position(not_blocked=True)), x_goal=Node(world.random_position(not_blocked=True)), gui=True, **filtered_args)
  farrt_star.run()