import argparse
from collections import defaultdict
import math
from queue import PriorityQueue, Queue
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
    self.consistence_eps = kwargs.pop('consistence_eps', 0.01) # no larger than 1/2 robot width
    
    super().__init__(*args, **kwargs)

    self.iters = max(self.iters, 5000)
    self.orphans: MultiPoint = MultiPoint()
    
    self.cost_to_goal: defaultdict[vertex_t, float] = defaultdict(lambda: float('inf'))
    self.lookahead_estimate: dict[vertex_t, float] = defaultdict(lambda: float('inf'))
    
    self.traj_cost_map: dict[edge_t, float] = defaultdict(lambda: float('inf')) # d_pi from RRTx paper

    self.inconsistencyPQ: PriorityQueue = PriorityQueue()
    self.queue_key_map: dict[vertex_t,tuple[float,float]] = dict() # needed to provide `set` API for the PQ

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
    removed_pts, closest_parents, freed_pts = self.do_tree_severing(previous_plan)
    print(f'removed: {len(removed_pts.geoms)}')
    print(f'closest: {len(closest_parents.geoms)}')
    print(f'freed: {len(freed_pts.geoms)}')

    # update potential field based on new obstacles
    self.update_potential_field(new_obstacles)

    # push free points with the potential field
    #   different from proposal - only field updating for free points (not whole tree)
    pushed_pts = self.apply_potential_field(self.orphans.union(freed_pts))
    # merge very close points into one
    pushed_pts = self.merge_points(pushed_pts)

    # set internal freed points to the field-updated free points
    self.orphans = multipoint_without(pushed_pts, self.detected_obstacles)

    # rewire the free points into the tree until curr pos is reached
    final_pt,final_cost = self.do_rrtx_rewiring(goal=self.curr_pos.coord, initial_frontier=closest_parents)
    
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
      self.set_child_parent(child=pt, parent=None)

    children = list(self.get_children(pt))
    for other in children:
      self.rrt_edges.discard((vtx, other))
      self.rrt_edges.discard((other, vtx))
      self.set_child_parent(child=other, parent=None)
    
    self.rrt_vertices.discard(vtx)
    self.rrt_tree = multipoint_without(self.rrt_tree, as_point(pt))

    self.set_cost_to_goal(pt, float('inf'))

  def do_tree_severing(self, previous_plan: list[Node]) -> tuple[MultiPoint, MultiPoint, MultiPoint]:
    """
    Sever nodes within obstacles and get the descendants as free points
    Also get the most recent parents to the deleted nodes
    """
    # get the points that are within the obstacles (or within half the obstacle avoidance radius of them)
    conflicting_pts = as_multipoint(self.rrt_tree.intersection(self.detected_obstacles.buffer(self.obstacle_avoidance_radius/2)))
    # add points that have an edge which is not obstacle free
    for node in previous_plan:
      if node.parent is not None and node.edgeToParent().intersects(self.detected_obstacles):
        conflicting_pts = conflicting_pts.union(node.coord)

    # determine the closest parents of the conflicting points - defaults to boundary zone around removed points
    closest_parents: MultiPoint = multipoint_without(self.rrt_tree.intersection(self.detected_obstacles.buffer(self.obstacle_avoidance_radius)), conflicting_pts)
    # build a Q for freeing all descendants of conflicting points
    severed_children_q: Queue[vertex_t] = Queue()
    for pt in conflicting_pts.geoms:
      closest_parents = closest_parents.union(self.get_parent(pt))
      severed_children_q.put(pt2tuple(pt))

    # if self.gui:
    #   print('Severing tree... - conflicting points in white')
    #   self.render(visualize=True, save_frame=True, extra_points=conflicting_pts)

    # ensure that none of the closest parents are in the conflicting points
    closest_parents = closest_parents - conflicting_pts

    # use Q to get all the descendants of the conflicting points
    severed_children: set[vertex_t] = set()
    while not severed_children_q.empty():
      point = severed_children_q.get()
      severed_children.add(point) # add pt to set
      children = self.get_children(point) # get children of pt
      new_children = children - severed_children # shouldnt be necessary
      for child in new_children: # add children to Q
        severed_children_q.put(child)
    
    # if self.gui:
    #   print('find children to sever - nodes to be severed in white')
    #   self.render(visualize=True, save_frame=True, extra_points=as_multipoint(severed_children))

    # actually sever all children from the tree/vertex sets
    for child in severed_children:
      self.sever_connections(child)

    # make only points actually in obstacles get deleted, the rest can be freed
    conflicting_pts = as_multipoint(conflicting_pts.intersection(self.detected_obstacles))

    # the freed points are all the severed nodes except conflicting points (conflicting points are deletec)
    freed_pts = multipoint_without(severed_children, conflicting_pts)

    # ensure that none of the closest parents are in the freed points
    closest_parents = as_multipoint(closest_parents - freed_pts)

    # render the tree with the severing results
    if self.gui:
      # print('Tree severing :: dark->light = closest parents -> conflicting -> freed')
      self.render(save_frame=True, extra_points={
        'moccasin': freed_pts,
        'darkorange': conflicting_pts,
        'peru': closest_parents,
      })

    return conflicting_pts, closest_parents, freed_pts

  def update_potential_field(self, new_obstacles: MultiPolygon, /):
    """
    Update the potential field based on new obstacles
    Convert the obstacle into a binary mask as a numpy array
    blur the mask and take te gradient of the blurred mask to get directions of force for the field update
    """
    print('Updating potential field...')
    obstacle_value = 5
    blur_sigma = 3

    # update potential field based on new obstacles
    obstacle_mask = np.ones(tuple(self.world.dims))

    # get a mask of all coordinates contained within the new obstacles
    obstacle: Polygon
    for obstacle in new_obstacles.geoms:
      minx, miny, maxx, maxy = obstacle.bounds
      for x in range(math.floor(minx), math.ceil(maxx)+1):
        for y in range(math.floor(miny), math.ceil(maxy)+1):
          if obstacle.contains(Point(x, y)):
            obstacle_mask[y,x] = 0
    
    # blur the mask
    blurred_mask = np.array(gaussian_filter(obstacle_mask*obstacle_value, sigma=blur_sigma), dtype=float)

    # take the gradient of the blurred mask
    dy,dx = np.gradient(blurred_mask)

    self.potential_field[:,:,0] += dx
    self.potential_field[:,:,1] += dy

    if self.gui:
      # print(f'Potential field updated with new obstacle')
      self.render(save_frame=True, potential_field=self.potential_field)

  def apply_potential_field(self, freed_pts: MultiPoint, /,*,field_force:float=None,tree_force:float = None,goal_force:float=None) -> MultiPoint:
    """
    push the given free points based on the potential field from the obstacles
    """
    print('Applying potential field to free points...')
    if field_force is None:
      field_force = self.potential_field_force  
    if tree_force is None:
      tree_force = self.tree_attr_force
    if goal_force is None:
      goal_force = self.goal_attr_force
    pushed_pts: list[Point] = []

    def get_movement_from_potential_field(pt: Point) -> tuple[float,float]:
      xInd = min(max(0, math.floor(pt.x)), self.world.dims[0]-1)
      yInd = min(max(0, math.floor(pt.y)), self.world.dims[1]-1)
      dx,dy = self.potential_field[yInd,xInd] * field_force
      nearby_pts = self.find_nearby_pts(pt, radius=self.steer_distance * 1.5, pt_source=self.rrt_tree)
      if not nearby_pts.is_empty:
        avg_tree_point = nearby_pts.centroid
        diff = np.array([avg_tree_point.x - pt.x, avg_tree_point.y - pt.y])
        diff *= tree_force / np.linalg.norm(diff)
        dx += diff[0]
        dy += diff[1]
      goal_diff = np.array([self.x_goal_pt.x - pt.x, self.x_goal_pt.y - pt.y])
      goal_diff *= goal_force / np.linalg.norm(goal_diff)
      dx += goal_diff[0]
      dy += goal_diff[1]
      return dx,dy

    # if self.gui:
    #   print('Before field update - freed points in white')
    #   self.render(visualize=True, extra_points=freed_pts)

    # for every free point, find the closest obstacle and push it away from the obstacle
    pt: Point
    for pt in freed_pts.geoms:
      dx,dy = get_movement_from_potential_field(pt)
      # if dx > 0 or dy > 0:
      #   print(f'Pushing point {pt} away from obstacles - {dx},{dy}')
      pushed_pts.append(Point(pt.x + dx, pt.y + dy))

    if self.gui:
      # print('After field update - freed:white  pushed:purple')
      self.render(save_frame=True, extra_points={
        'white': freed_pts,
        'purple': pushed_pts
      })

    return as_multipoint(pushed_pts)
  
  def merge_points(self, points: MultiPoint, merge_threshold: float = None) -> MultiPoint:
    """
    merge points that are very close together
    """
    print('Merging points...')
    if merge_threshold is None:
      merge_threshold = self.merge_threshold
    merged_pts: list[Point] = []

    # buffer all the points by half the merge threshold (create a collection of circles centered at each point)
    buffered = as_multipolygon(points.buffer(merge_threshold / 2))

    # take the centroid of every resulting polygon in the buffered shape
    #   points within the merge threshold will have merged circles, so they will only result in a single larger polygon in the buffered shape
    for poly in buffered.geoms:
      merged_pts.append(poly.centroid)
    
    print(f'Merged {len(points.geoms)} points into {len(merged_pts)} points')
    if self.gui:
      # print(f'After merging - points:white  merged:cyan')
      self.render(save_frame=True, extra_points={
        'white': points,
        'cyan': merged_pts,
      })

    # convert the list of points to a multipoint
    return as_multipoint(merged_pts)

  def sample_free_pts(self, frontier: MultiPoint, /,*, goal_pt: Point, frontier_radius:float = None) -> Point:
    """
    sample points that are free and not in the tree
    """
    if random.random() < self.eps * 2: # some higher % chance of returning goal node
      return goal_pt
    if frontier_radius is None:
      frontier_radius = self.steer_distance * 0.75
    local_free_pts = as_multipoint(self.orphans.intersection(frontier.buffer(frontier_radius)))
    while local_free_pts.is_empty:
      print('No free points found in local area, expanding search radius...')
      frontier_radius *= 1.5
      local_free_pts = as_multipoint(self.orphans.intersection(frontier.buffer(frontier_radius)))
    return random.choice(local_free_pts.geoms)

  def do_rrtx_rewiring(self, /,*, goal:Point, initial_frontier: MultiPoint):
    """
    Rewire the free points into the tree
    Sample from self.free_points based on the passed in initial_frontier (closest_parents)
      (the frontier of nodes which were severed)
    """
    print('Rewiring free points...')
    if self.gui:
      print(f'Start rewiring :: thistle - initial fronter')
      self.render(save_frame=True, extra_points={
        'thistle': initial_frontier
      })

    curr_vtx = pt2tuple(self.curr_pos)
    final_pt = None
    final_pt_cost = float('inf')
    
    frontier = initial_frontier

    i = 0
    while not self.orphans.is_empty:# and (curr_vtx not in self.rrt_vertices):
      if self.display_every_n >= 1 and i % (self.display_every_n*2) == 0:
        print(f"FARRT rewiring iteration {i}")
        if self.gui and i > 400 and i % 100 == 0:
          print(f"FARRT rewiring iteration {i} - frontier in white")
          self.render(visualize=True, extra_points=frontier)

      # sample a node from free points, find the nearest existing node, and steer from nearest to sampled
      x_free = self.sample_free_pts(frontier, goal_pt=goal)
      x_nearest = self.find_nearest(x_free, pt_source=self.rrt_tree)
      x_new = self.steer(x_nearest, x_free)
      if x_new != x_free: # if the new point is not the same as the sampled point
        print(f"Steering from {x_nearest} to {x_free} to get {x_new} - frontier in white")
        allow_for_far_goal = x_free == goal and not goal.buffer(self.steer_distance).intersects(self.orphans)
        if i > 99:
          if allow_for_far_goal:
            print('failed to steer to goal, must be too far! - allow new node creation')
          # self.render(visualize=True, extra_points=frontier)
        if not allow_for_far_goal:
          continue

      # if there is an obstacle free path from the nearest node to the new node, analyze neighbors and add to tree
      if self.edge_obstacle_free(x_nearest, x_new):
        # find nearby points to the new point
        nearby_points = self.find_nearby_pts(x_new, radius=self.find_ball_radius(num_vertices=len(self.rrt_vertices)), pt_source=self.rrt_tree)

        # get the minimum point from the set
        x_min,min_cost = self.get_min_cost_point(nearby_points, x_nearest, x_new)

        # add the new point to the tree
        self.add_vertex(pt=x_new,parent=x_min,cost=min_cost)
        self.orphans = self.orphans - x_new
        frontier = (frontier - x_min).union(x_new)

        # Main difference between RRT and RRT*, modify the points in the nearest set to optimise local path costs.
        self.do_rrtstar_rewiring(nearby_points, x_min, x_new)

        # check if we've reached the goal of the tree building
        if self.reached_goal(x_new, goal=goal, threshold=0):
          if self.built_tree: # subsequent runs should just terminate once goal is reached
            final_pt = x_new
            break
          else: # keep searching and update the shortest path
            if min_cost < final_pt_cost:
              final_pt = x_new
              final_pt_cost = min_cost
      i += 1

    return final_pt,final_pt_cost
  
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
      'free_points': as_multipoint(self.orphans),
      'rrt_children': self.parent_to_children_map,
    }


if __name__=='__main__':
  main(RRTX2)