import argparse
import math
from queue import Queue
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
from farrt.world import World
from farrt.utils import as_multipoint, as_multipolygon, as_point, multipoint_without, pt2tuple, shapely_edge


vertex_t = tuple[float,float]
edge_t = tuple[vertex_t,vertex_t]

class FARRTStar(RRTStar):

  def __init__(self, *args, **kwargs) -> None:
    self.merge_threshold = kwargs.pop('merge_threshold', None)
    self.potential_field_force = kwargs.pop('potential_field_force', 5)
    super().__init__(*args, **kwargs)
    if self.merge_threshold is None:
      self.merge_threshold = self.steer_distance / 8

    self.iters = max(self.iters, 4000)

    self.free_points: MultiPoint = MultiPoint()
    self.potential_field = np.zeros((*self.world.dims, len(self.world.dims)))

  def do_first_plan(self) -> None:
    if not self.detected_obstacles.is_empty:
      self.update_potential_field(self.detected_obstacles)
    super().do_first_plan()

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
    pushed_pts = self.apply_potential_field(self.free_points.union(freed_pts))
    # merge very close points into one
    pushed_pts = self.merge_points(pushed_pts)

    # set internal freed points to the field-updated free points
    self.free_points = multipoint_without(pushed_pts, self.detected_obstacles)

    # rewire the free points into the tree until curr pos is reached
    final_pt,final_cost = self.do_farrt_rewiring(goal=self.curr_pos.coord, initial_frontier=closest_parents)
    
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

    self.set_cost_to_reach(pt, float('inf'))

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

  def apply_potential_field(self, freed_pts: MultiPoint, /,*,field_force:float=None) -> MultiPoint:
    """
    push the given free points based on the potential field from the obstacles
    """
    print('Applying potential field to free points...')
    if field_force is None:
      field_force = self.potential_field_force  
    pushed_pts: list[Point] = []

    def get_movement_from_potential_field(pt: Point) -> tuple[float,float]:
      xInd = min(max(0, math.floor(pt.x)), self.world.dims[0]-1)
      yInd = min(max(0, math.floor(pt.y)), self.world.dims[1]-1)
      dx,dy = self.potential_field[yInd,xInd] * field_force
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
    # print('Sampling free points...')
    # sample points that are free and not in the tree
    # free_pts = self.free_pts.sample(self.free_pts_sample_size)
    # return np.random.choice(self.free_points.geoms)
    if random.random() < self.eps: # some % chance of returning goal node
      return goal_pt
    if frontier_radius is None:
      frontier_radius = self.steer_distance * 0.75
    local_free_pts = as_multipoint(self.free_points.intersection(frontier.buffer(frontier_radius)))
    while local_free_pts.is_empty:
      print('No free points found in local area, expanding search radius...')
      frontier_radius *= 1.5
      local_free_pts = as_multipoint(self.free_points.intersection(frontier.buffer(frontier_radius)))
    return random.choice(local_free_pts.geoms)

  def do_farrt_rewiring(self, /,*, goal:Point, initial_frontier: MultiPoint):
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
    while not self.free_points.is_empty:# and (curr_vtx not in self.rrt_vertices):
      if self.display_every_n >= 1 and i % (self.display_every_n*2) == 0:
        print(f"FARRT rewiring iteration {i}")
        if self.gui and i > 99 and i % 100 == 0:
          print(f"FARRT rewiring iteration {i} - frontier in white")
          self.render(visualize=True, extra_points=frontier)

      # sample a node from free points, find the nearest existing node, and steer from nearest to sampled
      x_free = self.sample_free_pts(frontier, goal_pt=goal)
      x_nearest = self.find_nearest(x_free)
      x_new = self.steer(x_nearest, x_free)
      if x_new != x_free: # if the new point is not the same as the sampled point
        print(f"Steering from {x_nearest} to {x_free} to get {x_new} - frontier in white")
        allow_for_far_goal = x_free == goal and not goal.buffer(self.steer_distance).intersects(self.free_points)
        if i > 99:
          if allow_for_far_goal:
            print('failed to steer to goal, must be too far! - allow new node creation')
          self.render(visualize=True, extra_points=frontier)
        if not allow_for_far_goal:
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
  
  def get_render_kwargs(self) -> dict:
    return {
      **super().get_render_kwargs(),
      'free_points': as_multipoint(self.free_points),
      'rrt_children': self.parent_to_children_map,
    }
  
  # def render_potential_field(self, **kwargs):
  #   if self.potential_field is None:
  #     return
    
  #   if self.force_no_visualization:
  #     return

  #   plot_potential_field(self.potential_field, **kwargs)


if __name__=='__main__':
  OBSTACLE_STR = None#"MULTIPOLYGON (((4.975463609829284 45.22491861450658, 4.975463609829284 37.61151918890049, 0 37.61151918890049, 0 45.22491861450658, 4.975463609829284 45.22491861450658)), ((53.751974771284615 22.281117482663035, 49.12366408922094 22.281117482663035, 49.12366408922094 29.581682990503822, 48.037113905181194 29.581682990503822, 48.037113905181194 40.841043105928364, 48.50257698962987 40.841043105928364, 48.50257698962987 42.391197540579796, 48.69331498520387 42.391197540579796, 48.69331498520387 44.16663811524711, 57.22527542815868 44.16663811524711, 57.22527542815868 40.841043105928364, 59.296474020605736 40.841043105928364, 59.296474020605736 34.326551541135814, 68.05454120509992 34.326551541135814, 68.05454120509992 24.688264481541736, 65.55480116965047 24.688264481541736, 65.55480116965047 14.440531653748378, 64.3213143533801 14.440531653748378, 64.3213143533801 13.227293264289607, 69.60316688528106 13.227293264289607, 69.60316688528106 12.021002333725448, 75.3732087857855 12.021002333725448, 75.3732087857855 11.443530975900913, 77.60595445232606 11.443530975900913, 77.60595445232606 6.501188360798675, 75.3732087857855 6.501188360798675, 75.3732087857855 5.80431183038417, 69.15651828244421 5.80431183038417, 69.15651828244421 7.358742643851158, 63.73461626484261 7.358742643851158, 63.73461626484261 10.010497798112514, 61.627968133595964 10.010497798112514, 61.627968133595964 9.531844272911744, 52.43699108867626 9.531844272911744, 52.43699108867626 12.65344941137736, 46.95900256372349 12.65344941137736, 46.95900256372349 20.502178999778153, 53.751974771284615 20.502178999778153, 53.751974771284615 22.281117482663035), (58.416254145505846 26.243358052114232, 58.416254145505846 29.581682990503822, 57.53005111507274 29.581682990503822, 57.53005111507274 26.243358052114232, 58.416254145505846 26.243358052114232)), ((63.54927631892523 80.92147319637623, 63.54927631892523 78.51689675581282, 65.08267419893906 78.51689675581282, 65.08267419893906 82.87379490868308, 65.7893671048644 82.87379490868308, 65.7893671048644 83.46684810429004, 67.18857096219054 83.46684810429004, 67.18857096219054 89.96218213080746, 76.93079444553348 89.96218213080746, 76.93079444553348 88.8123979892342, 86.31989302384692 88.8123979892342, 86.31989302384692 88.16321597293236, 87.1684156982815 88.16321597293236, 87.1684156982815 92.26469296052932, 81.73839620432572 92.26469296052932, 81.73839620432572 99.75095003901993, 89.22465328281633 99.75095003901993, 89.22465328281633 94.39796489173298, 96.41122172980042 94.39796489173298, 96.41122172980042 91.216922352416, 100 91.216922352416, 100 77.72486447234554, 88.21067097775216 77.72486447234554, 88.21067097775216 78.59418456802561, 86.31989302384692 78.59418456802561, 86.31989302384692 76.85311477097011, 74.36060980558283 76.85311477097011, 74.36060980558283 80.21995864746452, 71.91045970705022 80.21995864746452, 71.91045970705022 77.34575550210421, 70.85541239831736 77.34575550210421, 70.85541239831736 77.10105670930479, 67.51577480400086 77.10105670930479, 67.51577480400086 71.81214702530563, 66.84581345040546 71.81214702530563, 66.84581345040546 70.06668958730612, 73.87641829942332 70.06668958730612, 73.87641829942332 62.92157077361694, 66.73129948573414 62.92157077361694, 66.73129948573414 68.15610725387295, 58.11366270990708 68.15610725387295, 58.11366270990708 71.09442727656345, 53.72223039911247 71.09442727656345, 53.72223039911247 72.50801788018057, 51.24397007117599 72.50801788018057, 51.24397007117599 79.89212553072485, 53.72223039911247 79.89212553072485, 53.72223039911247 80.92147319637623, 63.54927631892523 80.92147319637623)), ((5.655982646274307 21.37444530968945, 5.655982646274307 23.267961254784076, 14.765304541967454 23.267961254784076, 14.765304541967454 25.49033127555, 11.326666343029583 25.49033127555, 11.326666343029583 26.965286834072476, 8.804852819750309 26.965286834072476, 8.804852819750309 38.71503035695148, 20.554596342629317 38.71503035695148, 20.554596342629317 31.374759069860136, 25.572529334153426 31.374759069860136, 25.572529334153426 25.94079223960237, 24.048351141686176 25.94079223960237, 24.048351141686176 22.085284156945573, 20.128573093163226 22.085284156945573, 20.128573093163226 20.279725573556277, 16.275593318396062 20.279725573556277, 16.275593318396062 13.402483606782198, 16.998566833301318 13.402483606782198, 16.998566833301318 6.083614773050009, 17.57838913811507 6.083614773050009, 17.57838913811507 0, 10.241187652453695 0, 10.241187652453695 3.6712572429015413, 2.863091614623972 3.6712572429015413, 2.863091614623972 12.805161629685033, 5.602018185169815 12.805161629685033, 5.602018185169815 13.201517496077315, 5.655982646274307 13.201517496077315, 5.655982646274307 15.236436551553481, 2.962546844721583 15.236436551553481, 2.962546844721583 21.37444530968945, 5.655982646274307 21.37444530968945)), ((37.10363865870382 85.1646616398993, 46.10360217391416 85.1646616398993, 46.10360217391416 76.16469812468895, 40.959724249130076 76.16469812468895, 40.959724249130076 74.70345461536215, 34.91690396182325 74.70345461536215, 34.91690396182325 80.74627490266897, 37.10363865870382 80.74627490266897, 37.10363865870382 85.1646616398993)), ((40.803245026692004 94.31924350920694, 40.803245026692004 100, 49.6983184075848 100, 49.6983184075848 94.31924350920694, 40.803245026692004 94.31924350920694)), ((51.73754179472827 57.14983920196681, 47.04874224145125 57.14983920196681, 47.04874224145125 53.95485078013806, 35.576076342197695 53.95485078013806, 35.576076342197695 59.607005616934025, 33.681151721340676 59.607005616934025, 33.681151721340676 60.60678284527032, 25.51142093198076 60.60678284527032, 25.51142093198076 65.86559150133719, 19.104699849417152 65.86559150133719, 19.104699849417152 66.8864554381405, 12.903793341551705 66.8864554381405, 12.903793341551705 71.8749983433896, 12.119881193139083 71.8749983433896, 12.119881193139083 64.66830247126128, 16.436637102678542 64.66830247126128, 16.436637102678542 59.3814040378438, 11.149738669261064 59.3814040378438, 11.149738669261064 63.33530621155646, 8.451569525155676 63.33530621155646, 8.451569525155676 61.86209756218105, 6.559642002154655 61.86209756218105, 6.559642002154655 60.34929030550278, 8.809309087999644 60.34929030550278, 8.809309087999644 49.423536912822016, 8.71184973738522 49.423536912822016, 8.71184973738522 48.14764609025869, 0 48.14764609025869, 0 71.34450371663996, 2.8628739734705446 71.34450371663996, 2.8628739734705446 72.592313431225, 6.970275758084061 72.592313431225, 6.970275758084061 78.51569943379202, 13.610976848486473 78.51569943379202, 13.610976848486473 77.31727036425411, 19.195619198341607 77.31727036425411, 19.195619198341607 78.6176920677674, 29.549966986531604 78.6176920677674, 29.549966986531604 69.20693745910171, 33.681151721340676 69.20693745910171, 33.681151721340676 69.4437089035844, 39.204729981176804 69.4437089035844, 39.204729981176804 73.3999696830151, 46.05679502251002 73.3999696830151, 46.05679502251002 66.54790464168188, 43.51785500799105 66.54790464168188, 43.51785500799105 65.824234791776, 51.73754179472827 65.824234791776, 51.73754179472827 57.14983920196681)), ((67.7958827501633 99.26546005221455, 67.7958827501633 92.33090778024395, 62.13661764878445 92.33090778024395, 62.13661764878445 86.11249458612232, 54.63575264468385 86.11249458612232, 54.63575264468385 93.61335959022293, 60.8613304781927 93.61335959022293, 60.8613304781927 99.26546005221455, 67.7958827501633 99.26546005221455)), ((64.68012346576447 51.013636317305334, 64.68012346576447 59.70582216796078, 73.37230931641992 59.70582216796078, 73.37230931641992 51.013636317305334, 64.68012346576447 51.013636317305334)), ((79.59585349238989 50.698136250033, 75.41618916276718 50.698136250033, 75.41618916276718 58.95843131181884, 76.41683042867821 58.95843131181884, 76.41683042867821 62.12285369823376, 78.81750683402282 62.12285369823376, 78.81750683402282 68.11718370424964, 84.98919582118837 68.11718370424964, 84.98919582118837 72.14538499878623, 93.62493919387602 72.14538499878623, 93.62493919387602 63.50964162609859, 88.4026157067826 63.50964162609859, 88.4026157067826 59.397257815989484, 89.32156040639582 59.397257815989484, 89.32156040639582 51.97988766302613, 91.76934196741651 51.97988766302613, 91.76934196741651 44.09564114350674, 88.72610055760009 44.09564114350674, 88.72610055760009 44.01297460031977, 98.85218974520612 44.01297460031977, 98.85218974520612 33.787398792764186, 92.87002434173186 33.787398792764186, 92.87002434173186 31.60285918972297, 91.91373718326187 31.60285918972297, 91.91373718326187 25.620289824709104, 85.02554803265186 25.620289824709104, 85.02554803265186 20.15013691932551, 84.82397850293655 20.15013691932551, 84.82397850293655 18.55244966527199, 78.82621263719176 18.55244966527199, 78.82621263719176 14.950950477605328, 69.51120354592015 14.950950477605328, 69.51120354592015 24.26595956887692, 73.4941700665767 24.26595956887692, 73.4941700665767 31.681514885400667, 80.73966584192758 31.681514885400667, 80.73966584192758 32.888332966057185, 78.70785172773827 32.888332966057185, 78.70785172773827 34.86386562977198, 78.1867120601261 34.86386562977198, 78.1867120601261 42.48512150546436, 80.14690243526064 42.48512150546436, 80.14690243526064 46.11902307777054, 79.59585349238989 46.11902307777054, 79.59585349238989 50.698136250033)), ((40.24799893371426 44.26236921741602, 40.24799893371426 41.839164371195416, 44.93627484211951 41.839164371195416, 44.93627484211951 36.43540274333379, 40.24799893371426 36.43540274333379, 40.24799893371426 35.22400979270988, 31.20963950900811 35.22400979270988, 31.20963950900811 44.26236921741602, 40.24799893371426 44.26236921741602)), ((97.49624132922231 18.520805138720284, 97.49624132922231 18.756567597301743, 100 18.756567597301743, 100 4.973839181870297, 91.5835995591346 4.973839181870297, 91.5835995591346 15.387253360656665, 93.36460672389194 15.387253360656665, 93.36460672389194 17.841258392519638, 94.52067881711913 17.841258392519638, 94.52067881711913 18.520805138720284, 97.49624132922231 18.520805138720284)), ((40.80372775809701 8.68467696768107, 32.83092054174769 8.68467696768107, 32.83092054174769 11.735363098232451, 28.867695473138273 11.735363098232451, 28.867695473138273 18.407920084065214, 35.54025245897104 18.407920084065214, 35.54025245897104 16.657484184030395, 40.80372775809701 16.657484184030395, 40.80372775809701 8.68467696768107)), ((77.60273431568822 95.92496041446839, 77.60273431568822 90.939751114394, 72.61752501561384 90.939751114394, 72.61752501561384 92.9060817934656, 70.77614183830703 92.9060817934656, 70.77614183830703 98.0268670869894, 75.89692713183082 98.0268670869894, 75.89692713183082 95.92496041446839, 77.60273431568822 95.92496041446839)), ((70.66389786007231 3.7423557465316377, 70.66389786007231 0, 64.82517992046253 0, 64.82517992046253 3.7423557465316377, 70.66389786007231 3.7423557465316377)), ((37.37171696850819 6.814318696334228, 46.769536912337806 6.814318696334228, 46.769536912337806 0, 37.37171696850819 0, 37.37171696850819 6.814318696334228)), ((13.081815810456265 50.06619758370228, 17.109961249408137 50.06619758370228, 17.109961249408137 48.581202144244486, 19.841830270355022 48.581202144244486, 19.841830270355022 41.91241294574688, 13.173041071857416 41.91241294574688, 13.173041071857416 46.03805214475041, 13.081815810456265 46.03805214475041, 13.081815810456265 50.06619758370228)), ((86.03708658817496 5.363437344651262, 86.03708658817496 10.030450102155456, 90.70409934567914 10.030450102155456, 90.70409934567914 5.363437344651262, 86.03708658817496 5.363437344651262)), ((5.159351139180918 100, 12.694662086680504 100, 12.694662086680504 92.59932987532616, 5.159351139180918 92.59932987532616, 5.159351139180918 100)), ((21.221528198708114 11.176798740987994, 31.763650131267312 11.176798740987994, 31.763650131267312 0.6346768084288001, 21.221528198708114 0.6346768084288001, 21.221528198708114 11.176798740987994)), ((82.86779007914978 0, 76.43502462096257 0, 76.43502462096257 4.278311689617949, 82.86779007914978 4.278311689617949, 82.86779007914978 0)), ((14.962612597789592 80.75240329185789, 9.421448044536328 80.75240329185789, 9.421448044536328 86.29356784511114, 14.962612597789592 86.29356784511114, 14.962612597789592 80.75240329185789)), ((95.68210819628422 94.44698075817445, 95.68210819628422 100, 100 100, 100 94.44698075817445, 95.68210819628422 94.44698075817445)), ((53.14417752308468 55.06822302236022, 53.14417752308468 63.06275938302354, 61.138713883748 63.06275938302354, 61.138713883748 55.06822302236022, 53.14417752308468 55.06822302236022)), ((69.60842585774793 38.88243300315011, 60.09909888594186 38.88243300315011, 60.09909888594186 48.39175997495619, 69.60842585774793 48.39175997495619, 69.60842585774793 38.88243300315011)))"

  world = World(obstacle_str=OBSTACLE_STR)
  print(world.obstacles)

  start = world.random_position(not_blocked=False)#Point(99.25924832620859, 65.55707024255547)
  goal = world.random_position(not_blocked=False)#Point(36.34354864552062, 97.4867297995134)
  
  random.seed(1)
  np.random.seed

  if OBSTACLE_STR is None:
    fig,ax = plot_world(world)
    start = world.random_position(not_blocked=True)
    goal = world.random_position(not_blocked=True)
    plot_point(ax, start, marker=".", markersize=6, markeredgecolor="red", markerfacecolor="red")
    plot_point(ax, goal, marker=".", markersize=6, markeredgecolor="green", markerfacecolor="green")
    plt.show()


  farrt_star = FARRTStar(world=world, x_start=Node(start), x_goal=Node(goal), gui=True)
  farrt_star.run()