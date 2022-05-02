import argparse
from collections import defaultdict
import math
from queue import PriorityQueue, Queue
import queue
import random
from matplotlib import pyplot as plt
import numpy as np
from shapely.geometry import Point, LineString, MultiPoint, MultiPolygon, Polygon
from shapely.geometry.base import BaseGeometry
from shapely.ops import nearest_points

from scipy.ndimage import gaussian_filter

from farrt.node import Node
from farrt.plot import plot_point, plot_potential_field, plot_world
from farrt.rrtstar import RRTStar
from farrt.runner import main
from farrt.world import World
from farrt.utils import as_multipoint, as_multipolygon, as_point, line2tuple, multipoint_without, pt2tuple, shapely_edge


vertex_t = tuple[float,float]
edge_t = tuple[vertex_t,vertex_t]

class RRTX2(RRTStar):

  def __init__(self, *args, **kwargs) -> None:
    self.consistence_eps = kwargs.pop('consistence_eps', 0.001) # no larger than 1/2 robot width
    
    super().__init__(*args, **kwargs)

    self.iters = max(self.iters, 5000)
    # self.orphans: MultiPoint = MultiPoint()
    
    self.neighbor_radius_map: dict[vertex_t, float] = defaultdict(lambda: self.shrinking_ball_radius)
    self.cost_to_goal: defaultdict[vertex_t, float] = defaultdict(lambda: float('inf'))
    self.lookahead_estimate: dict[vertex_t, float] = defaultdict(lambda: float('inf'))
    
    self.traj_cost_map: dict[edge_t, float] = defaultdict(lambda: float('inf')) # d_pi from RRTx paper

    self.inconsistencyPQ: PriorityQueue = PriorityQueue()
    self.queue_key_map: dict[vertex_t,tuple[float,float]] = dict() # needed to provide `set` API for the PQ

    self.shrinking_ball_radius = self.steer_distance#self.find_ball_radius(0)

  def update_shrinking_ball_radius(self, /) -> None:
    """
    Update the shrinking ball radius based on the goal point
    """
    new_radius = self.find_ball_radius(num_vertices=len(self.rrt_vertices))
    if new_radius != self.shrinking_ball_radius:
      self.shrinking_ball_radius = new_radius
      print(f"shrinking ball radius: {self.shrinking_ball_radius}")
    return self.shrinking_ball_radius

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
    self.set_lmc(root, 0)
    self.set_neighbor_radius(root, self.steer_distance)

    final_pt = None
    final_pt_cost = float('inf')

    # iterate until max iterations is reached or goal is reached
    i = 0
    while i < self.iters or final_pt is None:
      if self.display_every_n >= 1 and i % (self.display_every_n*2) == 0:
        print(f"RRT building iteration {i}")
        # if self.gui and i > 3000 and i % 1000 == 0:
        #   self.render(visualize=True)

      self.update_shrinking_ball_radius()

      # sample a node, find the nearest existing node, and steer from nearest to sampled
      v_rand = self.sample_free(goal_pt, require_free=False)
      v_nearest = self.find_nearest(v_rand, pt_source=self.rrt_tree)
      v_new = self.steer(v_nearest, v_rand)

      # if there is an obstacle free path from the nearest node to the new node, analyze neighbors and add to tree
      if self.point_obstacle_free(v_new):
        # find nearby points to the new point
        nearby_points = self.find_nearby_pts(v_new, radius=self.shrinking_ball_radius + self.consistence_eps, pt_source=self.rrt_tree)

        # get the optimal parent for the new node from the nearby points
        parent,new_lmc = self.find_rrtx_parent(nearby_points, v_new)
        if parent is not None:
          # print(f'found parent {parent}')
          # add the new point to the tree
          self.add_vertex(pt=v_new,parent=parent,cost=new_lmc)
          self.set_lmc(v_new, new_lmc)

          # set the neighborhood size of the new point
          self.set_neighbor_radius(v_new, self.shrinking_ball_radius)

          # Main difference between RRT and RRT*, modify the points in the nearest set to optimise local path costs.
          self.rewire_rrtx_neighbors(v_new)
          self.reduce_inconsistency()

          # check if we've reached the goal of the tree building
          if self.reached_goal(v_new, goal=goal_pt, threshold=goal_threshold):
            # update the shortest path
            if new_lmc < final_pt_cost:
              final_pt = v_new
              final_pt_cost = new_lmc
      i += 1
    return final_pt,final_pt_cost

  def find_rrtx_parent(self, V_near: MultiPoint, v_new: Point) -> tuple[Point,float]:
    """
    Finds the nearest point in the nearby points to be the parent of the new point
    """
    parent = None
    min_lmc = self.get_lmc(v_new)
    for u_nearby in V_near.geoms:
      if v_new == u_nearby: # pi(v,u) =/= {empty set}
        continue
      # if pt2tuple(u_nearby) not in self.rrt_vertices: # not sure if this right
      #   continue
      if not self.edge_obstacle_free(v_new, u_nearby):
        # print(f'Edge {v_new} -> {u_nearby} is not obstacle free')
        continue
      traj = (u_nearby, v_new)
      traj_cost = self.get_traj_cost(traj) # d_pi(v,u)
      cost_from_u = traj_cost + self.get_lmc(u_nearby)
      # print(f'{u_nearby}: traj={traj_cost}/{self.shrinking_ball_radius}, lmc={cost_from_u}/{self.get_lmc(v_new)}')
      if min_lmc > cost_from_u: # skip check for traj_cost < radius b/c guaranteed by being within nearby points
        # self.set_child_parent(child=v_new, parent=u_nearby)
        # self.set_lmc(v_new, cost_from_u)
        parent = u_nearby
        min_lmc = cost_from_u
    return parent, min_lmc

  def rewire_rrtx_neighbors(self, v_new: Point) -> None:
    """
    Rewire the neighbors of v_new to optimise the local path costs
    """
    if self.get_cost_to_goal(v_new) - self.get_lmc(v_new) > self.consistence_eps:
      self.cull_neighbors(v_new, self.shrinking_ball_radius)
      parent = self.get_parent(v_new)
      for u_in in multipoint_without(self.get_in_neighbors(v_new), parent):
        cost_through_v = self.get_traj_cost((u_in, v_new)) + self.get_lmc(v_new)
        if self.get_lmc(u_in) > cost_through_v:
          self.set_lmc(u_in, cost_through_v)
          self.reassign_parent(child=u_in, parent=v_new, cost=None)
          if self.get_cost_to_goal(u_in) - self.get_lmc(u_in) > self.consistence_eps:
            # print(f'{u_in} is inconsistent')
            self.verify_queue(u_in)

  def cull_neighbors(self, v_new: Point, radius: float) -> None:
    """
    Remove all running neighbors of v_new that are not obstacle free
    This method can be ignored because running neighbors are calculated on the fly
    """
    pass

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
        self.rewire_rrtx_neighbors(v_next)
      self.set_cost_to_goal(v_next, self.get_lmc(v_next))

  def update_lmc(self, pt: Point, /) -> None:
    """
    Update the lmc of a node
    based on Algo13: updateLMC(v) from RRTx paper
    """
    self.cull_neighbors(pt, self.shrinking_ball_radius)
    p_prime = None
    for u_out in multipoint_without(self.get_out_neighbors(pt, radius=self.shrinking_ball_radius), self.get_orphan_pts()).geoms: # N+(v) \ V^c_T
      if self.get_parent(u_out, allow_none=True) == as_point(pt): # p+_T(u) != v
        continue
      if self.get_lmc(pt) > self.get_traj_cost((pt, u_out)) + self.get_lmc(u_out):
        p_prime = u_out
    if p_prime is not None:
      self.reassign_parent(child=p_prime, parent=pt, cost=None)
      self.set_lmc(p_prime, self.get_traj_cost((pt, p_prime)) + self.get_lmc(p_prime))

  def replan(self, new_obstacles: MultiPolygon, **kwargs):
    """
    Replan the path from the current position to the goal.
    1. Sever nodes within obstacles
    2. Apply potential field update to remaining points
    3. Rewire free points into tree
    4. Extract a plan from the tree
    """
    if self.gui:
      # print('Render observations...')
      self.render(draw_world_obstacles=False, save_frame=True, **kwargs)
    print('Planning...')

    previous_plan = self.planned_path
    self.planned_path = []

    # sever nodes within obstacles and get the descendants as free points
    # removed_pts, closest_parents, freed_pts = 
    self.do_tree_severing(previous_plan)
    # print(f'removed: {len(removed_pts.geoms)}')
    # print(f'closest: {len(closest_parents.geoms)}')
    # print(f'freed: {len(freed_pts.geoms)}')

    # set internal freed points to the field-updated free points
    # self.orphans = multipoint_without(pushed_pts, self.detected_obstacles)

    self.verify_queue(self.curr_pos)

    self.reduce_inconsistency()
    final_pt = self.curr_pos.coord

    # # rewire the free points into the tree until curr pos is reached
    # final_pt,final_cost = self.do_rrtx_rewiring(goal=self.curr_pos.coord)
    
    # extract a plan from the tree and reverse it (to go from goal to start)
    self.planned_path = self.extract_path(endpoint=final_pt,root=self.x_goal_pt,reverse=True)

    print('Planning complete')
    # if self.gui:
    #   self.render()

  def sever_connections(self, pt: vertex_t):
    """
    Remove point from tree and cut all edges to/from the point
    Makes `pt` a free point
    """
    vtx = pt2tuple(pt)

    parent = self.get_parent(pt, allow_none=True)
    if parent is not None:
      parent_vtx = pt2tuple(parent)
      self.rrt_edges.discard((vtx, parent_vtx))
      self.rrt_edges.discard((parent_vtx, vtx))
      # self.set_child_parent(child=pt, parent=None)

    children = list(self.get_children(pt))
    for other in children:
      self.rrt_edges.discard((vtx, other))
      self.rrt_edges.discard((other, vtx))
      self.set_child_parent(child=other, parent=None)
    
    self.rrt_vertices.discard(vtx)
    self.rrt_tree = multipoint_without(self.rrt_tree, as_point(pt))

    # self.set_cost_to_goal(pt, float('inf'))

  def do_tree_severing(self, previous_plan: list[Node]) -> tuple[MultiPoint, MultiPoint, MultiPoint]:
    """
    Sever nodes within obstacles and get the descendants as free points
    Also get the most recent parents to the deleted nodes
    """
    print('Severing tree...')
    # get the points that are within the obstacles (or within half the obstacle avoidance radius of them)
    # conflicting_pts = as_multipoint(self.rrt_tree.intersection(self.detected_obstacles))
    # add points that have an edge which is not obstacle free
    # for node in previous_plan:
    #   if node.parent is not None and node.edgeToParent().intersects(self.detected_obstacles):
    #     # print(f'{pt2tuple(node)} is within an obstacle')
    #     conflicting_pts = conflicting_pts.union(node.parent.coord)
    #   self.set_traj_cost(v, float('inf'))

    orphans: set[vertex_t] = set()
    print(f'num edges to check: {len(self.rrt_edges)}')
    for rrt_edge in list(self.rrt_edges):
      edges = [rrt_edge,rrt_edge[::-1]]
      for edge in edges:
        if not self.edge_obstacle_free(*edge):
          print(f'edge {edge} is not obstacle free')
          self.set_traj_cost(edge, float('inf'))
          v,u = edge
          v_parent = self.get_parent(v, allow_none=True)
          if v_parent is None or v_parent == as_point(u):
            print(f'{u=} is parent of {v=} - mark v as orphan')
            # self.verify_orphan(v)
            self.removePQ(v)
            orphans.add(v)
            self.sever_connections(v)
          else:
            print(f'{u=} is not parent of {v=}, {v_parent=}')
            # self.render(visualize=True, extra_points={
            #   'purple': [u],
            #   'pink': [v],
            #   'teal': [v_parent],
            # })
          if self.curr_pos.coord.intersects(shapely_edge(*edge)):
            print(f'Current position intersects edge {edge}')
            pass # set pi_bot = 0?

    # determine the closest parents of the conflicting points - defaults to boundary zone around removed points
    # closest_parents: MultiPoint = multipoint_without(self.rrt_tree.intersection(self.detected_obstacles.buffer(self.obstacle_avoidance_radius)), conflicting_pts)
    # build a Q for freeing all descendants of conflicting points
    # severed_children_q: Queue[vertex_t] = Queue()
    # for pt in conflicting_pts.geoms:
    #   # closest_parents = closest_parents.union(self.get_parent(pt))
    #   severed_children_q.put(pt2tuple(pt))

    # if self.gui:
    #   print('Severing tree... - conflicting points in white')
    #   self.render(visualize=True, save_frame=True, extra_points=conflicting_pts)

    # ensure that none of the closest parents are in the conflicting points
    # closest_parents = closest_parents - conflicting_pts

    # use Q to get all the descendants of the conflicting points
    # severed_children: set[vertex_t] = set()
    # while not severed_children_q.empty():
    initial_orphans = list(orphans)
    for point in initial_orphans:
      # point = severed_children_q.get()
      # orphans.add(point) # add pt to set
      children = self.get_children(point) # get children of pt
      # new_children = children - orphans # shouldnt be necessary
      orphans.update(children) # add children to set
      # for child in new_children: # add children to Q
      #   # severed_children_q.put(child)
      #   orphans.add(child)
    
    # if self.gui:
    #   print('find children to sever - nodes to be severed in white')
    #   self.render(visualize=True, save_frame=True, extra_points=as_multipoint(severed_children))

    # actually sever all children from the tree/vertex sets
    # for child in orphans:
    #   self.sever_connections(child)

    # add the orphans to the rewiring queue
    # for child in severed_children:
    #   if not self.point_obstacle_free(child):
    #     continue
    #   self.insertPQ(child)
    for v in orphans:
      # self.removePQ(v)
      v_parent = self.get_parent(v, allow_none=True)
      parent_set = set() if v_parent is None else {pt2tuple(v_parent)}
      for u in (self.get_out_neighbors(v) | parent_set) - self.orphans:
        self.set_cost_to_goal(u, float('inf'))
        self.verify_queue(u)

    for v in orphans:
      # self.remove_orphan_vtx(v)
      # self.insert_tree_vtx(v, add_to_graph=False)
      self.set_cost_to_goal(v, float('inf'))
      self.set_lmc(v, float('inf'))
      parent = self.get_parent(v, allow_none=True)
      if parent != None:
        self.reassign_parent(child=v, parent=None, cost=None)
    
    self.rrt_tree = multipoint_without(self.rrt_tree, as_multipoint(orphans))
    # self.orphans = set()

    # make only points actually in obstacles get deleted, the rest can be freed
    # conflicting_pts = as_multipoint(conflicting_pts.intersection(self.detected_obstacles))

    # the freed points are all the severed nodes except conflicting points (conflicting points are deletec)
    # freed_pts = multipoint_without(orphans, conflicting_pts)

    # ensure that none of the closest parents are in the freed points
    # closest_parents = as_multipoint(closest_parents - freed_pts)

    # render the tree with the severing results
    if self.gui:
      # print('Tree severing :: dark->light = closest parents -> conflicting -> freed')
      self.render(save_frame=True, extra_points={
        'moccasin': as_multipoint(orphans),
        'darkorange': initial_orphans,
        # 'moccasin': freed_pts,
        # 'darkorange': conflicting_pts,
        # 'peru': closest_parents,
      })

    # return conflicting_pts, closest_parents, freed_pts

  def do_rrtx_rewiring(self, /,*, goal:Point):
    """
    Rewire the free points into the tree
    Sample from self.free_points based on the passed in initial_frontier (closest_parents)
      (the frontier of nodes which were severed)
    """
    print('Rewiring free points...')
    if self.gui:
      print(f'Start rewiring :: thistle - initial fronter')
      self.render(save_frame=True, extra_points={
        'thistle': []
      })

    curr_vtx = pt2tuple(self.curr_pos)
    final_pt = None
    final_pt_cost = float('inf')
    
    def testQtop(v_bot):
      """
      Test the top of the inconsistencyPQ against v_bot
      line 1 [keyLess] from Algo5: reduceInconsistency() from RRTx paper
      """
      top = self.popPQ()
      key_less = self.test_key_less(top, v_bot)
      self.insertPQ(top)
      return key_less

    i = 0
    while (self.queue_not_empty()\
      and (False
        or self.get_cost_to_goal(self.curr_pos) == float('inf') # g(v_bot) = inf
        or pt2tuple(self.curr_pos) in self.queue_key_map
        or testQtop(pt2tuple(self.curr_pos))
      )) or final_pt is None:
      if self.display_every_n >= 1 and i % (self.display_every_n*2) == 0:
        print(f"FARRT rewiring iteration {i}")
        if self.gui and i > 400 and i % 100 == 0:
          print(f"FARRT rewiring iteration {i}")
          self.render(visualize=True)


      self.update_shrinking_ball_radius()

      # sample a node from free points, find the nearest existing node, and steer from nearest to sampled
      # sample a node from free points, find the nearest existing node, and steer from nearest to sampled
      # x_free = self.sample_free_pts(frontier, goal_pt=goal)
      v_free = as_point(self.popPQ()) if self.queue_not_empty() else self.sample_free(goal, buffer_radius=0)
      v_nearest = self.find_nearest(v_free, pt_source=self.rrt_tree)
      v_new = self.steer(v_nearest, v_free)
      # if x_new != x_field: # if the new point is not the same as the sampled point
      #   print(f"Steering from {x_nearest} to {x_field} to get {x_new} - frontier in white")
      #   allow_for_far_goal = x_field == goal and not goal.buffer(self.steer_distance).intersects(self.free_points)
      #   if i > 99:
      #     if allow_for_far_goal:
      #       print('failed to steer to goal, must be too far! - allow new node creation')
      #     # self.render(visualize=True, extra_points=frontier)
      #   if not allow_for_far_goal:
      #     continue

      # if there is an obstacle free path from the nearest node to the new node, analyze neighbors and add to tree
      if self.point_obstacle_free(v_new):
        # find nearby points to the new point
        nearby_points = self.find_nearby_pts(v_new, radius=self.shrinking_ball_radius, pt_source=self.rrt_tree)

        # get the optimal parent for the new node from the nearby points
        parent,new_lmc = self.find_rrtx_parent(nearby_points, v_new)
        if parent is None:
          continue

        # add the new point to the tree
        self.add_vertex(pt=v_new,parent=parent,cost=new_lmc)
        self.set_lmc(v_new, new_lmc)

        # set the neighborhood size of the new point
        self.set_neighbor_radius(v_new, self.shrinking_ball_radius)

        # Main difference between RRT and RRT*, modify the points in the nearest set to optimise local path costs.
        self.rewire_rrtx_neighbors(v_new)
        self.reduce_inconsistency()

        # check if we've reached the goal of the tree building
        if self.reached_goal(v_new, goal=goal, threshold=0):
          # update the shortest path
          if new_lmc < final_pt_cost:
            final_pt = v_new
            final_pt_cost = new_lmc
      i += 1

    return final_pt,final_pt_cost
  

  def get_in_neighbors(self, v: Point, /,*, radius:float = None) -> MultiPoint:
    """
    Returns the set of in neighbors of v
    """
    v = as_point(v)
    if radius is None:
      self.get_neighbor_radius(v)
    return self.rrt_tree.union(self.get_orphan_pts()).intersection(v.buffer(radius))
  
  def get_out_neighbors(self, v: Point, /,*, radius:float = None) -> MultiPoint:
    """
    Returns the set of out neighbors of v
    """
    v = as_point(v)
    if radius is None:
      self.get_neighbor_radius(v)
    return self.rrt_tree.union(self.get_orphan_pts()).intersection(v.buffer(radius))

  def get_orphan_pts(self, /) -> MultiPoint:
    """
    Returns the set of orphan points
    """
    return as_multipoint(list(self.queue_key_map.keys()))

  def verify_queue(self, pt: Point, /) -> None:
    """
    Adds v to the queue if it is not already in the queue
    """
    pt = pt2tuple(pt)
    if pt in self.queue_key_map:
      self.updatePQ(pt)
    else:
      self.insertPQ(pt)

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

  def removePQ(self, pt: Point, /) -> None:
    """
    Remove v from the inconsistencyPQ
    """
    self.queue_key_map.pop(pt2tuple(pt), None)

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

  def set_neighbor_radius(self, pt: Point, radius: float, /) -> float:
    """
    Return the radius of the neighborhood around pt
    """
    self.neighbor_radius_map[pt2tuple(pt)] = radius

  def get_neighbor_radius(self, pt: Point, /) -> float:
    """
    Return the radius of the neighborhood around pt
    """
    return self.neighbor_radius_map[pt2tuple(pt)]

  def get_lmc(self, node: Point) -> float:
    return self.lookahead_estimate[pt2tuple(node)]
  
  def set_lmc(self, node: Point, lmc: float):
    self.lookahead_estimate[pt2tuple(node)] = lmc

  def get_traj_cost(self, edge):
    edge = tuple(sorted(line2tuple(edge)))
    return self.traj_cost_map.setdefault(edge, self.get_edge_cost(*edge))

  def set_traj_cost(self, edge, cost):
    edge = tuple(sorted(line2tuple(edge)))
    self.traj_cost_map[edge] = cost

  def get_render_kwargs(self) -> dict:
    return {
      **super().get_render_kwargs(),
      'free_points': self.get_orphan_pts(),
      'rrt_children': self.parent_to_children_map,
    }


if __name__=='__main__':
  obs = "MULTIPOLYGON (((77.98018699847196 48.890924263977666, 77.98018699847196 56.337998124260366, 80.61025492734979 56.337998124260366, 80.61025492734979 58.641893620692386, 78.94620764379944 58.641893620692386, 78.94620764379944 65.13100088335415, 84.23211592196931 65.13100088335415, 84.23211592196931 65.41640495500974, 85.49277807536407 65.41640495500974, 85.49277807536407 69.21990912844198, 89.07054947476675 69.21990912844198, 89.07054947476675 75.78971021239619, 87.97343013294632 75.78971021239619, 87.97343013294632 80.80840275593098, 92.99212267648112 80.80840275593098, 92.99212267648112 78.51935985942984, 98.75530816274625 78.51935985942984, 98.75530816274625 68.83460117145034, 90.23602159286852 68.83460117145034, 90.23602159286852 65.41640495500974, 91.1047148720486 65.41640495500974, 91.1047148720486 58.543806004930445, 85.96307644591067 58.543806004930445, 85.96307644591067 56.337998124260366, 89.542261550403 56.337998124260366, 89.542261550403 44.77592357232933, 79.00748548659598 44.77592357232933, 79.00748548659598 43.79751712329747, 81.06040908557229 43.79751712329747, 81.06040908557229 39.923610881731996, 85.75502306204302 39.923610881731996, 85.75502306204302 30.611357644000343, 77.73168816269443 30.611357644000343, 77.73168816269443 19.956937688670685, 66.8946208282358 19.956937688670685, 66.8946208282358 30.7940050231293, 76.44276982431137 30.7940050231293, 76.44276982431137 31.883561579635707, 76.36737925042237 31.883561579635707, 76.36737925042237 32.158816613843925, 73.13863861732922 32.158816613843925, 73.13863861732922 30.954166160495994, 67.60283136259899 30.954166160495994, 67.60283136259899 36.489973415226224, 72.7003665703187 36.489973415226224, 72.7003665703187 37.15134244926887, 73.16187992565399 37.15134244926887, 73.16187992565399 38.569179014087325, 68.68574023670564 38.569179014087325, 68.68574023670564 44.79626264391389, 64.8645604339525 44.79626264391389, 64.8645604339525 36.71773020520898, 53.158746581519324 36.71773020520898, 53.158746581519324 48.42354405764216, 53.234200092587315 48.42354405764216, 53.234200092587315 48.89832854068845, 44.09566638516624 48.89832854068845, 44.09566638516624 58.802388398784444, 53.99972624326222 58.802388398784444, 53.99972624326222 56.235609102689885, 61.20080990018164 56.235609102689885, 61.20080990018164 59.24767893377622, 65.03283046182534 59.24767893377622, 65.03283046182534 64.68725520759932, 66.26260695420987 64.68725520759932, 66.26260695420987 68.50401738475666, 77.34495277797257 68.50401738475666, 77.34495277797257 57.42167156099398, 75.71038347601407 57.42167156099398, 75.71038347601407 54.00970219341058, 71.63248435258103 54.00970219341058, 71.63248435258103 48.890924263977666, 77.98018699847196 48.890924263977666)), ((49.468689827608216 16.20126003346217, 49.468689827608216 6.076010693218409, 38.94600450681244 6.076010693218409, 38.94600450681244 16.598696014014187, 44.41006743211651 16.598696014014187, 44.41006743211651 23.476537761716457, 46.50038793408816 23.476537761716457, 46.50038793408816 28.033956015672636, 58.33308391629863 28.033956015672636, 58.33308391629863 16.20126003346217, 49.468689827608216 16.20126003346217)), ((0 93.35673979516416, 0 99.44072745811725, 4.868661380675365 99.44072745811725, 4.868661380675365 93.35673979516416, 0 93.35673979516416)), ((23.050971176307705 83.8210173178498, 20.81824335292373 83.8210173178498, 20.81824335292373 85.34862264261028, 20.37312788752331 85.34862264261028, 20.37312788752331 82.52253131553579, 14.213803093458798 82.52253131553579, 14.213803093458798 88.68185610960029, 17.363505375471107 88.68185610960029, 17.363505375471107 94.82931376233627, 15.76120148735365 94.82931376233627, 15.76120148735365 99.05776118027215, 19.989648905289535 99.05776118027215, 19.989648905289535 95.88032723712578, 27.895209969986624 95.88032723712578, 27.895209969986624 91.29311747325208, 33.417519562720535 91.29311747325208, 33.417519562720535 86.47908433885367, 35.99711481882477 86.47908433885367, 35.99711481882477 89.94387204179279, 43.89164499561646 89.94387204179279, 43.89164499561646 82.04934186500108, 37.62795275889709 82.04934186500108, 37.62795275889709 78.28978396908145, 29.43865238912489 78.28978396908145, 29.43865238912489 79.00155355017372, 25.631126017030716 79.00155355017372, 25.631126017030716 80.92656908683925, 23.050971176307705 80.92656908683925, 23.050971176307705 83.8210173178498)), ((84.6141615220117 92.43403031703808, 85.47342296494851 92.43403031703808, 85.47342296494851 95.71658138100312, 86.7182173180298 95.71658138100312, 86.7182173180298 100, 98.31523052881347 100, 98.31523052881347 94.56901328395648, 99.08666300661923 94.56901328395648, 99.08666300661923 93.49842248154893, 99.63071650520918 93.49842248154893, 99.63071650520918 84.06106358084536, 99.08666300661923 84.06106358084536, 99.08666300661923 83.45736221994412, 87.97501194260687 83.45736221994412, 87.97501194260687 85.51155454416917, 85.47342296494851 85.51155454416917, 85.47342296494851 87.80150251992295, 85.12231354716515 87.80150251992295, 85.12231354716515 85.7029696803328, 79.13201773915641 85.7029696803328, 79.13201773915641 91.69326548834154, 84.6141615220117 91.69326548834154, 84.6141615220117 92.43403031703808)), ((0.5594967329337708 29.054290938218347, 0.5594967329337708 38.19659020359191, 9.15922519274988 38.19659020359191, 9.15922519274988 38.31613567781571, 8.02247313107741 38.31613567781571, 8.02247313107741 48.0947662599947, 9.15922519274988 48.0947662599947, 9.15922519274988 48.15400973228169, 14.017026417357595 48.15400973228169, 14.017026417357595 55.852311347855284, 24.625159098232427 55.852311347855284, 24.625159098232427 45.24417866698045, 19.459391796847566 45.24417866698045, 19.459391796847566 37.85384312818401, 9.701795998307329 37.85384312818401, 9.701795998307329 29.054290938218347, 0.5594967329337708 29.054290938218347)), ((53.64836480985205 76.31077036378733, 53.64836480985205 84.00731592700986, 64.8132619646621 84.00731592700986, 64.8132619646621 72.84241877219979, 64.65808202031778 72.84241877219979, 64.65808202031778 71.84744381595281, 55.99798171579497 71.84744381595281, 55.99798171579497 72.84241877219979, 55.880801461468806 72.84241877219979, 55.880801461468806 71.10164597942361, 59.578593637490314 71.10164597942361, 59.578593637490314 61.83825123102174, 50.31519888908845 61.83825123102174, 50.31519888908845 67.1596577339813, 46.729688831662784 67.1596577339813, 46.729688831662784 76.31077036378733, 53.64836480985205 76.31077036378733)), ((77.9279216968029 7.381514894420175, 77.9279216968029 18.873130809469405, 89.41953761185214 18.873130809469405, 89.41953761185214 17.829316493748827, 97.59040508484543 17.829316493748827, 97.59040508484543 6.667790517692625, 86.42887910878923 6.667790517692625, 86.42887910878923 7.381514894420175, 77.9279216968029 7.381514894420175)), ((92.77443931214958 28.848021295846266, 92.77443931214958 39.40072498427401, 100 39.40072498427401, 100 28.848021295846266, 92.77443931214958 28.848021295846266)), ((24.491922362863008 2.150517903149498, 24.491922362863008 12.977835747640139, 35.31924020735365 12.977835747640139, 35.31924020735365 11.482733200255279, 36.77963350560457 11.482733200255279, 36.77963350560457 0.5963906980664078, 25.893291003415698 0.5963906980664078, 25.893291003415698 2.150517903149498, 24.491922362863008 2.150517903149498)), ((51.82327121708119 90.6546107359112, 44.29437834916575 90.6546107359112, 44.29437834916575 98.18350360382664, 44.56550658331211 98.18350360382664, 44.56550658331211 100, 53.123484341776766 100, 53.123484341776766 95.30900319780739, 51.82327121708119 95.30900319780739, 51.82327121708119 90.6546107359112)), ((5.088782628438379 84.77169683662218, 5.088782628438379 77.29650492864094, 0 77.29650492864094, 0 84.77169683662218, 5.088782628438379 84.77169683662218)), ((28.12557996533808 69.79138201369098, 29.580815044031333 69.79138201369098, 29.580815044031333 61.78437506007718, 21.573808090417543 61.78437506007718, 21.573808090417543 69.79138201369098, 22.039458211249116 69.79138201369098, 22.039458211249116 70.88491851599834, 28.12557996533808 70.88491851599834, 28.12557996533808 69.79138201369098)), ((2.9350022296922926 60.050214278838006, 2.9350022296922926 70.53414505104129, 13.418933001895587 70.53414505104129, 13.418933001895587 60.050214278838006, 2.9350022296922926 60.050214278838006)), ((11.778623522656304 90.16657554972596, 11.778623522656304 84.58306727657347, 6.195115249503804 84.58306727657347, 6.195115249503804 90.16657554972596, 11.778623522656304 90.16657554972596)), ((41.4576812467833 25.077094186359055, 31.216577461799467 25.077094186359055, 31.216577461799467 35.31819797134288, 37.86520301887402 35.31819797134288, 37.86520301887402 42.17660558545712, 41.05671556770504 42.17660558545712, 41.05671556770504 44.51778495829461, 42.41981471079389 44.51778495829461, 42.41981471079389 48.07782848466981, 50.36245016191344 48.07782848466981, 50.36245016191344 44.51778495829461, 52.87794586955499 44.51778495829461, 52.87794586955499 32.696554656444675, 41.4576812467833 32.696554656444675, 41.4576812467833 25.077094186359055)), ((78.61600283663337 92.91030381771562, 78.61600283663337 82.42016577759587, 68.12586479651362 82.42016577759587, 68.12586479651362 92.91030381771562, 78.61600283663337 92.91030381771562)), ((62.66137287084606 10.754432869467118, 57.33125317412006 10.754432869467118, 57.33125317412006 16.084552566193118, 62.66137287084606 16.084552566193118, 62.66137287084606 10.754432869467118)), ((28.908898631433907 53.17774585000669, 35.326024950546646 53.17774585000669, 35.326024950546646 46.76061953089395, 28.908898631433907 46.76061953089395, 28.908898631433907 53.17774585000669)), ((96.1445894202975 26.61466513779553, 100 26.61466513779553, 100 20.496998712227526, 96.1445894202975 20.496998712227526, 96.1445894202975 26.61466513779553)), ((3.5634931654748225 76.42562833300869, 3.5634931654748225 71.24515450227531, 0 71.24515450227531, 0 76.42562833300869, 3.5634931654748225 76.42562833300869)))"
  start = Point(9.385958677423488, 2.834747652200631)
  goal = Point(76.2280082457942, 0.2106053351110693)
  main(RRTX2)
  # main(RRTX2, obs, start, goal)
