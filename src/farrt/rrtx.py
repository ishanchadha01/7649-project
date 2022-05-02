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
from farrt.runner import main
from farrt.world import World
from farrt.utils import as_point, as_multipoint, line2tuple, multipoint_without, pt2tuple, shapely_edge

import imageio

vertex_t = tuple[float,float]
edge_t = tuple[vertex_t,vertex_t]

class RRTX(RRTBase):

  def __init__(self, *args, **kwargs) -> None:
    self.consistence_eps = kwargs.pop('consistence_eps', 0.01) # no larger than 1/2 robot width

    self.samples_per_turn = kwargs.pop('samples_per_turn', 500)

    super().__init__(*args, **kwargs)

    self.iters = max(self.iters, 3000)
    self.cost_to_goal: defaultdict[vertex_t, float] = defaultdict(lambda: float('inf'))

    self.rrtx_graph = MultiPoint()
    self.tree_vertices: set[vertex_t] = set() # V_T from RRTx paper
    self.orphans: set[vertex_t] = set() # V^c_T from RRTx paper

    self.perm_out_neighbors_map: dict[vertex_t, set[vertex_t]] = defaultdict(set) # N+_0 from RRTx paper
    self.running_out_neighbors_map: dict[vertex_t, set[vertex_t]] = defaultdict(set) # N+_r from RRTx paper

    self.perm_in_neighbors_map: dict[vertex_t, set[vertex_t]] = defaultdict(set) # N-_0 from RRTx paper
    self.running_in_neighbors_map: dict[vertex_t, set[vertex_t]] = defaultdict(set) # N-_r from RRTx paper

    self.lookahead_estimate: dict[vertex_t, float] = defaultdict(lambda: float('inf'))
    
    self.traj_cost_map: dict[edge_t, float] = defaultdict(lambda: float('inf')) # d_pi from RRTx paper

    self.inconsistencyPQ: PriorityQueue = PriorityQueue()
    self.queue_key_map: dict[vertex_t,tuple[float,float]] = dict() # needed to provide `set` API for the PQ

    self.shrinking_ball_radius = math.floor((math.prod(self.world.dims) * math.log(self.iters) / self.iters) ** (1/2)) # radius of ball for neighbors

  def setup_planner(self) -> None:
    """
    Initialize the RRTx graph with just the goal node in the tree set (root from goal and expand towards curr pos)
    """
    root = self.x_goal_pt
    self.insert_tree_vtx(root)
    self.set_cost_to_goal(root, 0)
    self.set_lmc(root, 0)

    for i in range(self.iters):
      # shrink ball radius
      N = len(self.rrtx_graph.geoms)
      # self.shrinking_ball_radius = math.floor((math.prod(self.world.dims) * math.log(N) / N) ** (1/2)) # radius of ball for neighbors
      self.shrinking_ball_radius = self.find_ball_radius(N)

      # print(f'Run update with N {N} - ball radius {self.shrinking_ball_radius}')

      # sample a new node and add it to the tree
      added = self.do_sampling()

      # if added:
      #   print(f' -> {len(self.rrtx_graph.geoms)}', end='')
    print(f'Final graph size: {len(self.rrtx_graph.geoms)}')
    if self.gui:
      self.render(save_frame=True)


  def handle_new_obstacles(self, new_obstacles: BaseGeometry) -> None:
    """
    Update internals based on the new obstacles
    based on algo11: addNewObstacle(O) in RRTx paper
    """
    # get the points that are within the obstacles (or within half the obstacle avoidance radius of them)
    conflicting_pts = as_multipoint(self.rrtx_graph.intersection(self.detected_obstacles))

    edges: set[edge_t] = set()
    for v_blocked in conflicting_pts.geoms:
      v_blocked = pt2tuple(v_blocked)
      if v_blocked not in self.tree_vertices:
        continue
      edges.update(map(lambda u_in: (u_in,v_blocked), filter(lambda u: u in self.tree_vertices, self.get_in_neighbors(v_blocked))))
      edges.update(map(lambda u_out: (v_blocked,u_out), filter(lambda u: u in self.tree_vertices, self.get_out_neighbors(v_blocked))))

    print(f'Conflicting pts: {len(conflicting_pts.geoms)}')
    print(f'Edges: {len(edges)}')

    for edge in edges: # Line 6 from Algo7: updateObstacles() in RRTx paper
      edge_geom = LineString(edge)
      # filter edges that intersect the new obstacles
      if not edge_geom.intersects(new_obstacles):
        # print(f'Edge {edge} does not intersect new obstacles')
        continue
      
      # print(f'Edge {edge} intersects new obstacles')

      # update the cost to goal for the nodes that are affected by the new obstacle
      self.set_traj_cost(edge, float('inf'))
      v,u = edge
      v_parent = self.get_parent(v, allow_none=True)
      if v_parent is None or v_parent == as_point(u):
        # print(f'{u=} is parent of {v=} - mark v as orphan')
        self.verify_orphan(v)
      if self.curr_pos.coord.intersects(edge_geom):
        print(f'Current position intersects edge {edge}')
        pass # set pi_bot = 0?
    
    self.propogate_descendants()
    self.verify_queue(self.curr_pos)
    self.reduce_inconsistency()

  def handle_deleted_obstacles(self, deleted_obstacles: MultiPolygon) -> None:
      return super().handle_deleted_obstacles(deleted_obstacles)

  def update_planner(self) -> None:
    pass

  def do_sampling(self) -> Point:
    """
    Sample a free node and add it to the RRTx
    Returns the new point if it was added to the tree
    Based on line 9-17 of Algo1: RRTx(X,S) in RRTx paper
    """
    v_rand = self.sample_free(goal_pt=self.curr_pos, require_free=False)
    v_nearest = self.find_nearest(v_rand, pt_source=self.rrtx_graph)
    v_new = self.steer(v_nearest, v_rand) # line 11-12 in Algo1: RRTx(X,S) in RRTx paper - saturate to delta (steer distance)

    # add the node to the rrtx graph and tree if obstacle free
    if self.point_obstacle_free(v_new):
      self.extend(v_new, self.shrinking_ball_radius + self.consistence_eps)
    if self.in_graph(v_new):
      # print(f'Added new node {v_new} to RRTx')
      self.rewire_neighbors(v_new)
      self.reduce_inconsistency()

      # if v_new == self.curr_pos.coord:
      #   print(f'Found curr pos at {v_new}')
      #   if self.get_cost_to_goal(v_new) < float('inf'):
      #     print(f'Cost: {self.get_cost_to_goal(v_new)} - extracting path...')
      #     self.render(visualize=True)
      #     path = self.extract_path(endpoint=v_new,root=self.x_goal_pt,reverse=True)
      #     self.planned_path = path
      #     print(f'Found path to goal: {len(path)} - {[pt2tuple(pt) for pt in path]}')

      return v_new
    return None

  def propogate_descendants(self):
    """
    Propogate the descendants of the current node
    cascades cost-to-goal increase throughout orphans after deletion when obstacle is added
    must ensure the decrease cascade from reduceInconsistency reaches relevant parts of T
    based on Algo8: propogateDescendants() in RRTx paper
    """
    # add all descendants of orphans to orphan tree too
    for o in list(self.orphans):
      self.orphans.update(self.get_children(o))
    # child_q: Queue = Queue()
    # for v in self.orphans:
    #   child_q.put(v)
    # while not child_q.empty():
    #   v = child_q.get()
    #   self.remove_tree_vtx(v)
    #   self.insert_orphan_vtx(v)
    #   children = self.get_children(v)
    #   new_children = children - self.orphans
    #   for child in new_children:
    #     child_q.put(child)

    for v in self.orphans:
      v_parent = self.get_parent(v, allow_none=True)
      parent_set = set() if v_parent is None else {pt2tuple(v_parent)}
      for u in (self.get_out_neighbors(v) | parent_set) - self.orphans:
        self.set_cost_to_goal(u, float('inf'))
        self.verify_queue(u)

    for v in self.orphans:
      # self.remove_orphan_vtx(v)
      # self.insert_tree_vtx(v, add_to_graph=False)
      self.set_cost_to_goal(v, float('inf'))
      self.set_lmc(v, float('inf'))
      parent = self.get_parent(v, allow_none=True)
      if parent != None:
        self.remove_child(parent=parent, child=v)
        self.remove_parent(child=v)
    
    self.rrtx_graph = multipoint_without(self.rrtx_graph, as_multipoint(self.orphans))
    self.orphans = set()

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
      # print(f'Finding parent for {v_new} from {V_near}')
      for u_nearby in V_near.geoms:
        if v_new == u_nearby: # pi(v,u) =/= {empty set}
          continue
        if pt2tuple(u_nearby) not in self.tree_vertices: # not sure if this right
          continue
        if not self.edge_obstacle_free(v_new, u_nearby):
          # print(f'Edge {v_new} -> {u_nearby} is not obstacle free')
          continue
        traj = (u_nearby, v_new)
        traj_cost = self.get_traj_cost(traj) # d_pi(v,u)
        cost_from_u = traj_cost + self.get_lmc(u_nearby)
        # print(f'{u_nearby}: traj={traj_cost}/{self.shrinking_ball_radius}, lmc={cost_from_u}/{self.get_lmc(v_new)}')
        if traj_cost < radius and self.get_lmc(v_new) > cost_from_u:
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
    # print(f'Extended {v_new} to {parent=}')

    # add the new node to the tree
    self.insert_tree_vtx(v_new)
    self.add_child(parent=parent, child=v_new)

    # update neighbors
    for u_nearby in V_near.geoms:
      v_to_u = False
      u_to_v = False
      if self.edge_obstacle_free(v_new, u_nearby): # pi(v,u) is obstacle free
        v_to_u = True
        self.add_perm_out_neighbor(frm=v_new, to=u_nearby)
        self.add_running_in_neighbor(to=u_nearby, frm=v_new)
      if self.edge_obstacle_free(u_nearby, v_new): # pi(u,v) is obstacle free
        u_to_v = True
        self.add_running_out_neighbor(frm=u_nearby, to=v_new)
        self.add_perm_in_neighbor(frm=u_nearby, to=v_new)
      if v_to_u != u_to_v:
        print(f'{v_new} -> {u_nearby} is different obstacle freeness depending on direction')
        print(f'{v_to_u=} {u_to_v=}')
      # if v_to_u:
      #   print(f'{v_new} -> {u_nearby} is obstacle free')
      #   print(f'{self.get_running_out_neighbors(u_nearby)=}')
      #   print(f'{self.get_running_in_neighbors(u_nearby)=}')
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
          self.set_parent(child=u_in, parent=v_new)
          if self.get_cost_to_goal(u_in) - self.get_lmc(u_in) > self.consistence_eps:
            # print(f'{u_in} is inconsistent')
            self.verify_queue(u_in)

  def cull_neighbors(self, v, radius):
    """
    Remove all neighbors of v that are outside the ball centered at v
    only allow edges shorter than r (except edges in T)
    based on Algo3: cullNeighbors(v,r) from RRTx paper
    """
    for u_out_running in list(self.get_running_out_neighbors(v)):
      traj = LineString([v, u_out_running])
      if radius < self.get_traj_cost(traj) and self.get_parent(v) != as_point(u_out_running):
        # print(f'Removing {u_out_running=} from {v=}')
        # print(f'{self.get_running_out_neighbors(v)=}')
        # print(f'{self.get_running_in_neighbors(u_out_running)=}')
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
      # if key_less:
      #   print(f'{top=} has key less {v_bot=}')
      # else:
      #   print(f'{top=} has key greater {v_bot=}')
      return key_less

    while self.queue_not_empty()\
      and (False
        or self.get_lmc(self.curr_pos) != self.get_cost_to_goal(self.curr_pos) # lmc(v_bot) != g(v_bot)
        or self.get_cost_to_goal(self.curr_pos) == float('inf') # g(v_bot) = inf
        or pt2tuple(self.curr_pos) in self.queue_key_map
        or testQtop(pt2tuple(self.curr_pos))
      ):
      # print(f'{self.queue_not_empty()=}')
      v_next = self.popPQ()
      # print(f'Reducing inconsistency of {v_next=}')
      # print(f'{self.get_lmc(v_next)=} {self.get_cost_to_goal(v_next)=}')
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
    # self.queue_key_map.pop(pt, None)
    self.tree_vertices.discard(pt)
    self.orphans.add(pt)

  def verify_queue(self, pt: Point, /) -> None:
    """
    Verify a point is in the queue of inconsistent points
    based on Algo12: verrifyQueue(v) from RRTx paper
    """
    pt = pt2tuple(pt)
    if pt in self.queue_key_map:
      self.updatePQ(pt)
    else:
      self.insertPQ(pt)

  def update_lmc(self, pt: Point, /) -> None:
    """
    Update the lmc of a node
    based on Algo13: updateLMC(v) from RRTx paper
    """
    self.cull_neighbors(pt, self.shrinking_ball_radius)
    p_prime = None
    for u_out in self.get_out_neighbors(pt) - self.orphans: # N+(v) \ V^c_T
      if self.get_parent(u_out, allow_none=True) == as_point(pt): # p+_T(u) != v
        continue
      if self.get_lmc(pt) > self.get_traj_cost((pt, u_out)) + self.get_lmc(u_out):
        p_prime = u_out
    if p_prime is not None:
      self.set_parent(child=pt, parent=p_prime)
      self.set_lmc(p_prime, self.get_traj_cost((pt, p_prime)) + self.get_lmc(p_prime))
    # else:
    #   print(f'no children for {pt}')

  def step_through_plan(self) -> Node:
    parent = self.get_parent(self.curr_pos, allow_none=True)
    if parent is not None:
      return Node(as_point(parent), self.curr_pos)
    children = set() if parent is None else {pt2tuple(parent)}
    print(f'Try stepping to one of {children=}')
    nearby = multipoint_without(self.rrtx_graph.intersection(self.curr_pos.coord.buffer(self.steer_distance*0.75)), self.curr_pos.coord)
    for pt in nearby.geoms:
      if self.edge_obstacle_free(self.curr_pos.coord, pt):
        children.add(pt2tuple(pt))
    print(f'\tw/ nearby {children=}')
    if len(children) == 0:
      return self.curr_pos
    # children = list(filter(lambda child: self.get_parent(child, allow_none=True) == as_point(self.curr_pos), children))
    children = list(filter(lambda child: self.edge_obstacle_free(self.curr_pos.coord, child), children))
    print(f'\tfiltered {children=}')
    if len(children) == 0:
      return self.curr_pos
    def child_key(child):
      key0,key1 = self.get_key(child)
      traj_cost = self.get_traj_cost((pt2tuple(self.curr_pos), child))
      return (key0+traj_cost, key1+traj_cost)
    min_child = min(children, key=lambda child: child_key(child))
    print(f'\tmin child {min_child=}')
    return Node(as_point(min_child), parent=self.curr_pos)

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
    key = self.get_key(pt)
    # only add it if not already present
    if pt not in self.queue_key_map:
      self.queue_key_map[pt] = key
      self.inconsistencyPQ.put((key,pt))
    else:
      print(f'Warning: {pt} already in inconsistencyPQ - {self.queue_key_map[pt]}')

  def updatePQ(self, pt: Point) -> None:
    """
    Update the key of v in the inconsistencyPQ
    """
    pt = pt2tuple(pt)
    key = self.get_key(pt)
    # only update it if it is already present
    if pt in self.queue_key_map:
      self.queue_key_map[pt] = key
      self.inconsistencyPQ.put((key,pt))
    else:
      print(f'Warning: {pt} not in inconsistencyPQ, tried to update - {key}')

  def popPQ(self) -> Point:
    """
    Pop the top of the inconsistencyPQ
    """
    key,point = self.inconsistencyPQ.get(block=False)
    # ensure that the point is still in the inconsistent set
    while point not in self.queue_key_map or self.queue_key_map[point] != key:
      point = self.inconsistencyPQ.get(block=False)
    self.queue_key_map.pop(point)
    return point

  def queue_not_empty(self) -> bool:
    """
    Return whether the inconsistencyPQ is not empty
    """
    try:
      # key,point = self.inconsistencyPQ.get(block=False)
      # self.inconsistencyPQ.put((key,point))
      point = self.popPQ()
      self.insertPQ(point)
      return True
    except queue.Empty:
      return False

  def insert_tree_vtx(self, point: Point, /,*, add_to_graph:bool = True) -> None:
    """
    inserts a new vertex into the tree
    """
    if add_to_graph:
      # print(f'Add {point} to graph')
      self.rrtx_graph = as_multipoint(self.rrtx_graph.union(as_point(point)))
      if len(self.rrtx_graph.geoms) % self.samples_per_turn == 0:
        print(f'Graph now has {len(self.rrtx_graph.geoms)} points')
        # if self.gui:
        #   self.render(visualize=True)
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
    return pt2tuple(point) in (self.tree_vertices | self.orphans)

  def set_parent(self, /,*, child: Point, parent: Point):
    child = pt2tuple(child)
    parent = pt2tuple(parent)
    if child == parent:
      raise ValueError(f'Child {child} is parent of itself')
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

  def get_running_in_neighbors(self, pt: Point, /) -> set[vertex_t]:
    return self.running_in_neighbors_map[pt2tuple(pt)]

  def get_out_neighbors(self, pt: Point, /) -> set[vertex_t]:
    pt = pt2tuple(pt)
    return self.running_out_neighbors_map[pt] | self.perm_out_neighbors_map[pt]

  def get_in_neighbors(self, pt: Point, /) -> set[vertex_t]:
    pt = pt2tuple(pt)
    return self.running_in_neighbors_map[pt] | self.perm_in_neighbors_map[pt]

  def remove_running_out_neighbor(self, /,*, frm: Point, to: Point):
    frm = pt2tuple(frm)
    to = pt2tuple(to)
    self.running_out_neighbors_map[frm].discard(to)

  def remove_running_in_neighbor(self, /,*, to: Point, frm: Point):
    to = pt2tuple(to)
    frm = pt2tuple(frm)
    self.running_in_neighbors_map[to].discard(frm)

  def get_traj_cost(self, edge):
    edge = tuple(sorted(line2tuple(edge)))
    return self.traj_cost_map.setdefault(edge, self.get_edge_cost(*edge))

  def set_traj_cost(self, edge, cost):
    edge = tuple(sorted(line2tuple(edge)))
    self.traj_cost_map[edge] = cost

  def get_render_kwargs(self) -> dict:
    return {
      'rrt_tree': self.rrtx_graph,
      'rrt_parents': self.child_to_parent_map,
      # 'extra_points': {
      #   'yellow': self.rrtx_graph,
      #   'orange': self.tree_vertices,
      #   'blue': self.orphans,
      # }
    }


if __name__=='__main__':
  main(RRTX)