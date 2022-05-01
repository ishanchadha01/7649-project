import argparse
from collections import defaultdict
import os
from copy import deepcopy
import queue
import random
import math
import shutil
from sqlite3 import NotSupportedError
import tkinter as tk
import numpy as np
from farrt.rrtbase import RRTBase
from farrt.rrtstar import RRTStar
from scipy.spatial import KDTree
from queue import PriorityQueue, Queue
from matplotlib import pyplot as plt

from shapely.geometry.base import BaseGeometry
from shapely.geometry import Point, LineString, MultiPoint, MultiPolygon
from shapely.ops import nearest_points

from farrt.PartiallyObservablePlanner import PartiallyObservablePlanner
from farrt.rrtstar import RRTStar
from farrt.node import Node
from farrt.plot import plot_polygons, plot_planner
from farrt.world import World
from farrt.utils import as_point, as_multipoint, line2tuple, multipoint_without, pt2tuple, shapely_edge

import imageio

vertex_t = tuple[float,float]
edge_t = tuple[vertex_t,vertex_t]

class RRTX(RRTBase):

  def __init__(self, *args, **kwargs) -> None:
    self.consistence_eps = kwargs.pop('consistence_eps', 0.1) # no larger than 1/2 robot width

    super().__init__(*args, **kwargs)

    self.cost_to_goal: defaultdict[vertex_t, float] = defaultdict(lambda: float('inf'))

    self.rrtx_graph = MultiPoint()
    self.tree_vertices: set[vertex_t] = set() # V_T from RRTx paper
    self.orphans: set[vertex_t] = set() # V^c_T from RRTx paper

    self.perm_out_neighbors_map: dict[vertex_t, set[vertex_t]] = defaultdict(set) # N+_0 from RRTx paper
    self.running_out_neighbors_map: dict[vertex_t, set[vertex_t]] = defaultdict(set) # N+_r from RRTx paper

    self.perm_in_neighbors_map: dict[vertex_t, set[vertex_t]] = defaultdict(set) # N-_0 from RRTx paper
    self.running_in_neighbors_map: dict[vertex_t, set[vertex_t]] = defaultdict(set) # N-_r from RRTx paper

    self.lookahead_estimate: dict[vertex_t, float] = defaultdict(float('inf'))
    
    self.traj_cost_map: dict[edge_t, float] = defaultdict(lambda: float('inf')) # d_pi from RRTx paper

    self.inconsistencyPQ: PriorityQueue = PriorityQueue()
    self.inconsistent_set: set[vertex_t] = set() # needed to provide `set` API for the PQ
    

    self.shrinking_ball_radius = math.floor((math.prod(self.world.dims) * math.log(self.iters) / self.iters) ** (1/2)) # radius of ball for neighbors

  def setup_planner(self) -> None:
    """
    Initialize the RRTx graph with just the goal node in the tree set (root from goal and expand towards curr pos)
    """
    root = self.x_goal_pt
    self.insert_tree_vtx(root)
    self.set_cost_to_goal(root, 0)

  def handle_new_obstacles(self, new_obstacles: BaseGeometry) -> None:
    """
    Update internals based on the new obstacles
    based on algo11: addNewObstacle(O) in RRTx paper
    """
    for edge in self.rrt_edges: # Line 6 from Algo7: updateObstacles() in RRTx paper
      edge_geom = LineString(edge)
      # filter edges that intersect the new obstacles
      if not edge_geom.intersects(new_obstacles):
        continue

      # update the cost to goal for the nodes that are affected by the new obstacle
      self.set_traj_cost(edge, float('inf'))
      v,u = edge
      if self.get_parent(v) == u:
        self.verify_orphan(v)
      if self.curr_pos.coord.intersects(edge_geom):
        pass # set pi_bot = 0?
    
    self.propogate_descendants()
    self.verify_queue(self.curr_pos)
    self.reduce_inconsistency()

  def update_plan(self) -> None:
    # if self.lookahead_estimate[self.curr_pos] != self.cost_to_goal[self.curr_pos]:
    #   nearby_pts = self.find_nearest(self.curr_pos, pt_source=self.rrtx_graph)
    #   v_nearest = self.find_nearby_pts(self.curr_pos, self.shrinking_ball_radius, pt_source=self.rrtx_graph)
    #   v_min, _ = self.get_min_cost_point(nearby_pts, v_nearest, self.curr_pos)
    #   self.do_rrtv_rewiring(nearby_pts, v_min, self.curr_pos)
    #   self.update_key(self.curr_pos)
    #   for neighbor in self.children_map[self.curr_pos]:
    #     if self.lookahead_estimate[self.curr_pos] != self.cost_to_goal[self.curr_pos]:
    #       self.update_key(neighbor)
    # self.reduce_inconsistency()
    
    # shrink ball radius
    self.shrinking_ball_radius = math.floor((math.prod(self.world.dims) * math.log(self.iters) / self.iters) ** (1/2)) # radius of ball for neighbors

    # sample a new node and add it to the tree
    self.do_sampling()

  def do_sampling(self):
    """
    Sample a free node and add it to the RRTx
    Based on line 9-17 of Algo1: RRTx(X,S) in RRTx paper
    """
    v_rand = self.sample_free(goal_pt=None, require_free=False)
    v_nearest = self.find_nearest(v_rand, pt_source=self.rrtx_graph)
    v_new = self.steer(v_nearest, v_rand) # line 11-12 in Algo1: RRTx(X,S) in RRTx paper - saturate to delta (steer distance)

    # add the node to the rrtx graph and tree if obstacle free
    if self.point_obstacle_free(v_new):
      self.extend(v_new, self.shrinking_ball_radius)
    if self.in_graph(v_new):
      self.rewire_neighbors(v_new)
      self.reduce_inconsistency()

  def propogate_descendants(self):
    """
    Propogate the descendants of the current node
    cascades cost-to-goal increase throughout orphans after deletion when obstacle is added
    must ensure the decrease cascade from reduceInconsistency reaches relevant parts of T
    based on Algo8: propogateDescendants() in RRTx paper
    """
    # add all descendants of orphans to orphan tree too
    child_q: Queue = Queue()
    for v in self.orphans:
      child_q.put(v)
    while not child_q.empty():
      v = child_q.get()
      self.remove_tree_vtx(v)
      self.insert_orphan_vtx(v)
      children = self.get_children(v)
      new_children = children - self.orphans
      for child in new_children:
        child_q.put(child)
      
    for v in self.orphans:
      for u in (self.get_out_neighbors(v) | {pt2tuple(self.get_parent(v))}) - self.orphans:
        self.set_cost_to_goal(u, float('inf'))
        self.verify_queue(u)
    
    for v in list(self.orphans):
      self.remove_orphan_vtx(v)
      self.insert_tree_vtx(v, add_to_graph=False)
      self.set_cost_to_goal(v, float('inf'))
      self.set_lmc(v, float('inf'))
      parent = self.get_parent(v, allow_none=True)
      if parent != None:
        self.remove_child(parent=parent, child=v)
        self.remove_parent(child=v)

  def extend(self, v_new, radius) -> bool:
    """
    Attempt to the tree towards v_new
    based on Algo2: extend(v,r) from RRTx paper
    Returns True if the node was added to the tree
    """

    def find_parent(v_new: Point, V_near: MultiPoint):
      """
      finds the best parent for v_new from the node set V_near
      based on Algo6: findParent(v,U) from RRTx paper
      """
      for u_nearby in V_near.geoms:
        if v_new == u_nearby: # pi(v,u) =/= {empty set}
          continue
        if not self.edge_obstacle_free(v_new, u_nearby):
          continue
        traj = LineString([v_new, u_nearby])
        traj_cost = self.get_traj_cost(traj) # d_pi(v,u)
        cost_from_u = traj_cost + self.get_lmc(u_nearby)
        if traj_cost < self.shrinking_ball_radius and self.get_lmc(v_new) > cost_from_u:
          self.set_parent(child=v_new, parent=u_nearby)
          self.set_lmc(v_new, cost_from_u)

    # get points near the new node
    V_near = self.find_nearby_pts(v_new, radius=radius, pt_source=self.rrtx_graph)
    # attempt to assign one of the nearby points as parent to the new node
    find_parent(v_new, V_near)

    # check if a parent was assigned
    parent = self.get_parent(v_new, allow_none=True)
    if parent is None:
      return False

    # add the new node to the tree
    self.insert_tree_vtx(v_new)
    self.add_child(parent, v_new)

    # update neighbors
    for u_nearby in V_near.geoms:
      if self.edge_obstacle_free(v_new, u_nearby): # pi(v,u) is obstacle free
        self.add_perm_out_neighbor(frm=v_new, to=u_nearby)
        self.add_running_in_neighbor(to=u_nearby, frm=v_new)
      if self.edge_obstacle_free(u_nearby, v_new): # pi(u,v) is obstacle free
        self.add_running_out_neighbor(frm=u_nearby, to=v_new)
        self.add_perm_in_neighbor(frm=u_nearby, to=v_new)
    return True

  def rewire_neighbors(self, v_new) -> None:
    """
    Rewire the neighbors of v_new to use v as parent when better
    make sure to queue inconsistent nodes to trigger a cascade of rewiring
    based on Algo4: rewireNeighbors(v) from RRTx paper
    """

    if self.get_cost_to_goal(v_new) - self.get_lmc(v_new) > self.consistence_eps:
      self.cull_neighbors(v_new, self.shrinking_ball_radius)
      parent = self.get_parent(v_new)
      for u_in in self.get_in_neighbors(v_new) - {pt2tuple(parent)}:
        cost_through_v = self.get_traj_cost((u_in, v_new)) + self.get_lmc(v_new)
        if self.get_lmc(u_in) > cost_through_v:
          self.set_lmc(u_in, cost_through_v)
          self.set_parent(child=v_new, parent=u_in)
          if self.get_cost_to_goal(u_in) - self.get_lmc(u_in) > self.consistence_eps:
            self.verify_queue(u_in)

  def cull_neighbors(self, v, radius):
    """
    Remove all neighbors of v that are outside the ball centered at v
    only allow edges shorter than r (except edges in T)
    based on Algo3: cullNeighbors(v,r) from RRTx paper
    """
    for u_out_running in self.get_running_out_neighbors(v):
      traj = LineString([v, u_out_running])
      if radius < self.get_traj_cost(traj) and self.get_parent(v) != as_point(u_out_running):
        self.remove_running_out_neighbor(frm=v, to=u_out_running)
        self.remove_running_in_neighbor(to=u_out_running, frm=v)

  def reduce_inconsistency(self) -> None:
    """
    Reduce the inconsistency of the tree - pop entries from the inconsistency queue in priority order
    propogates cost-to-goal info to neighbors to maintain consistency
    - only cascades when v_next is still inconsistent (better optimality)
    based on Algo5: reduceInconsistency() from RRTx paper
    """

    def testQtop(v_bot):
      """
      Test the top of the inconsistencyPQ against v_bot
      line 1 [keyLess] from Algo5: reduceInconsistency() from RRTx paper
      """
      top = self.popPQ()
      key_less = self.test_key_less(top, v_bot)
      self.insertPQ(top)
      return key_less

    while self.inconsistencyPQ.qsize() > 0\
      and (False
        or self.get_lmc(self.curr_pos) != self.get_cost_to_goal(self.curr_pos) # lmc(v_bot) != g(v_bot)
        or self.get_cost_to_goal(self.curr_pos) == float('inf') # g(v_bot) = inf
        or pt2tuple(self.curr_pos) in self.inconsistent_set
        or testQtop(self.inconsistencyPQ, pt2tuple(self.curr_pos))
      ):
      v_next = self.popPQ()
      if self.get_cost_to_goal(v_next) - self.get_lmc(v_next) > self.consistence_eps:
        self.update_lmc(v_next)
        self.rewire_neighbors(v_next)
      self.set_cost_to_goal(v_next, self.get_lmc(v_next))

  def verify_orphan(self, pt: Point, /) -> None:
    """
    Remove the point from the inconsistency tree, make it an orphan (not in main tree)
    based on Algo9: verrifyOrphan(v) from RRTx paper
    """
    pt = pt2tuple(pt)
    self.inconsistent_set.discard(pt)
    self.tree_vertices.discard(pt)
    self.orphans.add(pt)

  def verify_queue(self, pt: Point, /) -> None:
    """
    Verify a point is in the queue of inconsistent points
    based on Algo12: verrifyQueue(v) from RRTx paper
    """
    pt = pt2tuple(pt)
    if pt not in self.inconsistent_set:
      self.insertPQ(pt)

  def update_lmc(self, pt: Point, /) -> None:
    """
    Update the lmc of a node
    based on Algo13: updateLMC(v) from RRTx paper
    """
    self.cull_neighbors(pt, self.shrinking_ball_radius)
    p_prime = None
    for u_out in self.get_out_neighbors(pt) - self.orphans:
      if self.get_parent(u_out) == as_point(pt):
        continue
      if self.get_lmc(pt) > self.get_traj_cost((pt, u_out)) + self.get_lmc(u_out):
        p_prime = u_out
    assert p_prime is not None
    self.set_parent(child=p_prime, parent=pt)

  def step_through_plan(self) -> Node:
    children = self.get_children(self.curr_pos)
    if len(children) == 0:
      return self.curr_pos
    children = list(filter(lambda child: self.get_parent(child, allow_none=True) == as_point(self.curr_pos), children))
    if len(children) == 0:
      return self.curr_pos
    min_child = min(children, key=lambda child: self.get_key(child))
    return Node(min_child, parent=self.curr_pos)

  # def replan(self, new_obstacles: MultiPolygon, **kwargs):
  #   # raise NotSupportedError("RRTX does not support replanning - should be handled within other functions")
  #   conflicting_pts = as_multipoint(self.rrt_tree.intersection(self.detected_obstacles))
  #   # get rid of points in obstacles
  #   # remove edges to their neighbors
  #   # if their neighbors arent in obstacle, add to orphan set
  #   # propagate this change to all descendants of these orphan nodes
  #   # for every orphan and all of its descendants, set cost to reach to inf
  #   # for every orphan and all of its descendants, add to PQ/inconsistent set, with key being their previous LMC + half the perimeter of the obstacle
  #   # since the update to the lookahead estimate is at least that much]
  #   # sample from the orphan set based on lookahead values, prolly bidirectional search from curr pos and path among orphan nodes
  #   # if time, do aco every time we run into an obstacle to avoid dead ends, similar to reasoning for APF for farrt
  #   self.update_key(self.curr_pos)
  #   self.reduce_inconsistency()

  def get_key(self, pt: Point) -> tuple[float,float]:
    """
    Return the key of pt
    """
    g = self.get_cost_to_goal(pt)
    lmc = self.get_lmc(pt)
    return (min(g, lmc), g)

  def test_key_less(self, v_1, v_2) -> bool:
    """
    Test if v_1 is less than v_2 in the key ordering
    keyLess for Algo5: reduceInconsistency() from RRTx paper
    """
    return self.get_key(v_1) < self.get_key(v_2)

  def insertPQ(self, pt: Point) -> None:
    """
    Insert v into the inconsistencyPQ
    """
    pt = pt2tuple(pt)
    # only add it if not already present
    if pt not in self.inconsistent_set:
      self.inconsistent_set.add(pt)
      self.inconsistencyPQ.put(pt)
  
  def popPQ(self) -> Point:
    """
    Pop the top of the inconsistencyPQ
    """
    point = self.inconsistencyPQ.get()
    # ensure that the point is still in the inconsistent set
    while point not in self.inconsistent_set:
      point = self.inconsistencyPQ.get()
    self.inconsistent_set.remove(point)
    return point

  def insert_tree_vtx(self, point: Point, /,*, add_to_graph:bool = True) -> None:
    """
    inserts a new vertex into the tree
    """
    if add_to_graph:
      self.rrtx_graph = self.rrtx_graph.union(as_point(point))
    self.tree_vertices.add(pt2tuple(point))

  def remove_tree_vtx(self, point: Point) -> None:
    """
    removes a vertex from the tree
    """
    self.tree_vertices.discard(pt2tuple(point))
  
  def insert_orphan_vtx(self, point: Point) -> None:
    """
    inserts a new vertex into the orphan set
    """
    self.orphans.add(pt2tuple(point))

  def remove_orphan_vtx(self, point: Point) -> None:
    """
    removes a vertex from the orphan set
    """
    self.orphans.discard(pt2tuple(point))

  def in_graph(self, point: Point) -> bool:
    return pt2tuple(point) in self.rrt_vertices

  def set_parent(self, /,*, child: Point, parent: Point):
    child = pt2tuple(child)
    parent = pt2tuple(parent)
    self.child_to_parent_map[child] = parent

  def remove_parent(self, /,*, child: Point) -> None:
    child = pt2tuple(child)
    if child in self.child_to_parent_map:
      del self.child_to_parent_map[child]

  def add_child(self, /,*, parent: Point, child: Point) -> None:
    """
    adds a child to the parent
    """
    parent = pt2tuple(parent)
    child = pt2tuple(child)
    self.parent_to_children_map[parent].add(child)

  def remove_child(self, /,*, parent: Point, child: Point) -> None:
    """
    removes a child from the parent
    """
    parent = pt2tuple(parent)
    child = pt2tuple(child)
    self.parent_to_children_map[parent].discard(child)

  def get_lmc(self, node: Point) -> float:
    return self.lookahead_estimate[pt2tuple(node)]
  
  def set_lmc(self, node: Point, lmc: float):
    self.lookahead_estimate[pt2tuple(node)] = lmc

  def add_perm_out_neighbor(self, /,*, frm: Point, to: Point):
    frm = pt2tuple(frm)
    to = pt2tuple(to)
    self.perm_out_neighbors_map[frm].add(to)

  def add_perm_in_neighbor(self, /,*, to: Point, frm: Point):
    to = pt2tuple(to)
    frm = pt2tuple(frm)
    self.perm_in_neighbors_map[to].add(frm)

  def add_running_out_neighbor(self, /,*, frm: Point, to: Point):
    frm = pt2tuple(frm)
    to = pt2tuple(to)
    self.running_out_neighbors_map[frm].add(to)
  
  def add_running_in_neighbor(self, /,*, to: Point, frm: Point):
    to = pt2tuple(to)
    frm = pt2tuple(frm)
    self.running_in_neighbors_map[to].add(frm)

  def get_running_out_neighbors(self, pt: Point, /) -> set[vertex_t]:
    return self.running_out_neighbors_map[pt2tuple(pt)]

  def get_out_neighbors(self, pt: Point, /) -> set[vertex_t]:
    pt = pt2tuple(pt)
    return self.running_out_neighbors_map[pt] | self.perm_out_neighbors_map[pt]

  def get_in_neighbors(self, pt: Point, /) -> set[vertex_t]:
    pt = pt2tuple(pt)
    return self.running_in_neighbors_map[pt] | self.perm_in_neighbors_map[pt]

  def remove_running_out_neighbor(self, /,*, frm: Point, to: Point):
    frm = pt2tuple(frm)
    to = pt2tuple(to)
    self.running_out_neighbors_map[frm].remove(to)

  def remove_running_in_neighbor(self, /,*, to: Point, frm: Point):
    to = pt2tuple(to)
    frm = pt2tuple(frm)
    self.running_in_neighbors_map[to].remove(frm)

  def get_traj_cost(self, edge):
    edge = tuple(sorted(line2tuple(edge)))
    self.traj_cost_map[edge]

  def set_traj_cost(self, edge, cost):
    edge = tuple(sorted(line2tuple(edge)))
    self.traj_cost_map[edge] = cost


if __name__=='__main__':
  world = World()
  rrtx = RRTX(world=world, x_start=Node(world.random_position(not_blocked=True)), x_goal=Node(world.random_position(not_blocked=True)), gui=True)
  rrtx.run()
