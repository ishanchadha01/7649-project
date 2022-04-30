import argparse
import math
from queue import Queue
import random
import numpy as np
from shapely.geometry import Point, LineString, MultiPoint, MultiPolygon, Polygon
from shapely.geometry.base import BaseGeometry
from shapely.ops import nearest_points

from scipy.ndimage import gaussian_filter

from farrt.node import Node
from farrt.rrtstar import RRTStar
from farrt.world import World
from farrt.utils import as_multipoint, as_multipolygon, as_point, multipoint_without, pt2tuple, shapely_edge


vertex_t = tuple[float,float]
edge_t = tuple[vertex_t,vertex_t]

class FARRTStar(RRTStar):

  def __init__(self, *args, **kwargs) -> None:
    self.merge_threshold = kwargs.pop('merge_threshold', 0.1)
    self.potential_field_force = kwargs.pop('potential_field_force', 0.1)
    super().__init__(*args, **kwargs)

    self.iters = max(self.iters, 4000)

    self.free_points: MultiPoint = MultiPoint()
    self.potential_field = np.zeros((*self.world.dims, len(self.world.dims)))

  def replan(self, new_obstacles: MultiPolygon, **kwargs):
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

    self.planned_path = []

    # sever nodes within obstacles and get the descendants as free points
    removed_pts, closest_parents, freed_pts = self.do_tree_severing()
    print(f'removed: {len(removed_pts)}')
    print(f'closest: {len(closest_parents)}')
    print(f'freed: {len(freed_pts)}')

    # update potential field based on new obstacles
    self.update_potential_field(new_obstacles)

    # push free points with the potential field
    #   different from proposal - only field updating for free points (not whole tree)
    pushed_pts = self.apply_potential_field(self.free_points.union(freed_pts))
    # merge very close points into one
    pushed_pts = self.merge_points(pushed_pts)

    # add the freed points to the internal set of free points - and make sure curr pos is present
    self.free_points = as_multipoint(pushed_pts.union(self.curr_pos.coord))

    # rewire the free points into the tree
    final_pt,final_cost = self.do_farrt_rewiring(closest_parents)
    
    # extract a plan from the tree and reverse it (to go from goal to start)
    self.planned_path = self.extract_path(endpoint=final_pt,root=self.x_goal_pt,reverse=True)

    print('Planning complete')
    if self.gui:
      self.render()

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

    self.set_cost_to_reach(pt, float('inf'))

  def do_tree_severing(self):
    conflicting_pts = as_multipoint(self.rrt_tree.intersection(self.detected_obstacles))
    closest_parents: set[vertex_t] = set()
    severed_children_q: Queue[vertex_t] = Queue()
    for pt in conflicting_pts.geoms:
      closest_parents.add(pt2tuple(self.get_parent(pt)))
      severed_children_q.put(pt2tuple(pt))

    severed_children: set[vertex_t] = set()
    while not severed_children_q.empty():
      severed_child = severed_children_q.get()
      severed_children.add(severed_child)
      children = self.get_children(severed_child)
      new_children = children - severed_children
      severed_children.update(new_children)
      for child in new_children:
        severed_children_q.put(child)
    
    for child in severed_children:
      self.sever_connections(child)

    freed_pts = multipoint_without(MultiPoint(list(severed_children)), conflicting_pts)

    return conflicting_pts, closest_parents, freed_pts

  def update_potential_field(self, new_obstacles: MultiPolygon):
    """
    Update the potential field based on new obstacles
    Convert the obstacle into a binary mask as a numpy array
    blur the mask and take te gradient of the blurred mask to get directions of force for the field update
    """
    print('Updating potential field...')
    # update potential field based on new obstacles
    blurred_mask = np.zeros(tuple(self.world.dims))

    # get a mask of all coordinates contained within the new obstacles
    obstacle: Polygon
    for obstacle in new_obstacles.geoms:
      minx, miny, maxx, maxy = obstacle.bounds
      for x in range(math.floor(minx), math.ceil(maxx)+1):
        for y in range(math.floor(miny), math.ceil(maxy)+1):
          if obstacle.contains(Point(x, y)):
            blurred_mask[x, y] = 1
    
    # blur the mask
    blurred_mask = np.array(gaussian_filter(blurred_mask, sigma=1), dtype=np.float)

    # take the gradient of the blurred mask
    dx,dy = np.gradient(blurred_mask)

    self.potential_field[:,:,0] += dx
    self.potential_field[:,:,1] += dy

  def apply_potential_field(self, freed_pts: MultiPoint, /,*,field_force:float=None) -> MultiPoint:
    """
    push the given free points based on the potential field from the obstacles
    """
    print('Applying potential field to free points...')
    if field_force is None:
      field_force = self.potential_field_force  
    pushed_pts: list[Point] = []

    def get_movement_from_closest_obstacle(pt: Point) -> tuple[float,float]:
      # find the closest obstacle
      closest_obstacle_pt = nearest_points(pt, self.detected_obstacles)[1]
      dist = pt.distance(closest_obstacle_pt)

      # push the point away from the obstacle
      movement = field_force / (dist*dist)
      dx = (pt.x - closest_obstacle_pt.x) * movement
      dy = (pt.y - closest_obstacle_pt.y) * movement
      return dx,dy

    def get_movement_from_potential_field(pt: Point) -> tuple[float,float]:
      dx,dy = self.potential_field[math.floor(pt.x), math.floor(pt.y)] * field_force
      return dx,dy

    # for every free point, find the closest obstacle and push it away from the obstacle
    pt: Point
    for pt in freed_pts.geoms:
      dx,dy = get_movement_from_potential_field(pt)
      pushed_pts.append(Point(pt.x + dx, pt.y + dy))

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
    
    # convert the list of points to a multipoint
    return as_multipoint(merged_pts)

  def sample_free_pts(self, frontier: MultiPoint, frontier_radius:float = None) -> Point:
    """
    sample points that are free and not in the tree
    """
    # print('Sampling free points...')
    # sample points that are free and not in the tree
    # free_pts = self.free_pts.sample(self.free_pts_sample_size)
    # return np.random.choice(self.free_points.geoms)
    if frontier_radius is None:
      frontier_radius = self.steer_distance * 1.5
    local_free_pts = as_multipoint(self.free_points.intersection(frontier.buffer(frontier_radius)))
    while local_free_pts.is_empty:
      frontier_radius *= 1.5
      local_free_pts = as_multipoint(self.free_points.intersection(frontier.buffer(frontier_radius)))
    return random.choice(local_free_pts.geoms)

  def do_farrt_rewiring(self, closest_parents: set[vertex_t]):
    """
    Rewire the free points into the tree
    Sample from self.free_points based on the passed in closest_parents
      (the frontier of nodes which were severed)
    """
    print('Rewiring free points...')

    curr_vtx = pt2tuple(self.curr_pos)
    final_pt = None
    final_pt_cost = float('inf')
    
    frontier = MultiPoint(list(closest_parents))

    i = 0
    while not self.free_points.is_empty:# and (curr_vtx not in self.rrt_vertices):
      if self.display_every_n >= 1 and i % (self.display_every_n*2) == 0:
        print(f"FARRT rewiring iteration {i}")
        # if self.gui and i > 1000 and i % 1000 == 0:
        #   self.render(visualize=True)

      # sample a node from free points, find the nearest existing node, and steer from nearest to sampled
      x_free = self.sample_free_pts(frontier)
      x_nearest = self.find_nearest(x_free)
      x_new = self.steer(x_nearest, x_free)
      if x_new != x_free: # if the new point is not the same as the sampled point
        # print(f"Steering from {x_nearest} to {x_rand} to get {x_new}")
        continue

      # if there is an obstacle free path from the nearest node to the new node, analyze neighbors and add to tree
      if self.edge_obstacle_free(x_nearest, x_new):
        # find nearby points to the new point
        nearby_points = self.find_nearby_pts(x_new)

        # get the minimum point from the set
        x_min,min_cost = self.get_min_cost_point(nearby_points, x_nearest, x_new)

        # add the new point to the tree
        self.add_vertex(pt=x_new,parent=x_min,cost=min_cost)
        self.free_points = self.free_points - x_new
        frontier = (frontier - x_min).union(x_new)

        # Main difference between RRT and RRT*, modify the points in the nearest set to optimise local path costs.
        self.do_rrtstar_rewiring(nearby_points, x_min, x_new)

        # check if we've reached the goal of the tree building
        if self.reached_goal(x_new, goal=self.curr_pos.coord, threshold=0):
          if self.built_tree: # subsequent runs should just terminate once goal is reached
            final_pt = x_new
            break
          else: # keep searching and update the shortest path
            if min_cost < final_pt_cost:
              final_pt = x_new
              final_pt_cost = min_cost
      i += 1

    return final_pt,final_pt_cost
  
  def get_render_kwargs(self) -> dict:
    return {
      **super().get_render_kwargs(),
      'free_points': as_multipoint(self.free_points),
    }


if __name__=='__main__':
  world = World()
  farrt_star = FARRTStar(world=world, x_start=Node(world.random_position(not_blocked=True)), x_goal=Node(world.random_position(not_blocked=True)), gui=True)
  farrt_star.run()