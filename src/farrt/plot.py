from collections import defaultdict
import numpy as np
from matplotlib.path import Path
from matplotlib.patches import PathPatch
from matplotlib.collections import PatchCollection

from shapely.geometry import Polygon, Point, MultiPoint
from shapely.geometry.base import BaseGeometry, BaseMultipartGeometry
import matplotlib.pyplot as plt
from farrt.node import Node
from farrt.utils import as_point, crop_field, normalize_field, pt2tuple

from farrt.world import World

def plot_point(ax, point: Point, **kwargs):
  point = as_point(point)
  if isinstance(point, Point):
    ax.plot(point.x, point.y, **kwargs)
  else:
    raise TypeError('plot_point() expects a Point, but got a {}'.format(type(point)))

def plot_line(ax, start: Point, end: Point, **kwargs):
  start = as_point(start)
  end = as_point(end)
  if isinstance(start, Point) and isinstance(end, Point):
    ax.plot([start.x, end.x], [start.y, end.y], **kwargs)
  else:
    raise TypeError('plot_line() expect two Points, but got a {} and {}'.format(type(start), type(end)))

def plot_points(points: list, parents_map:dict = None, children_map:defaultdict[set] = None, ax=None, **kwargs):
  if ax is None:
    fig,ax = plt.subplots()
  else:
    fig=None
  if isinstance(points, MultiPoint):
    points = points.geoms
  if isinstance(points, BaseMultipartGeometry):
    print(f'plot_points() expects a list of Points, but got a {type(points)}')
    points = points.geoms
  has_rrt_maps = parents_map is not None or children_map is not None
  if has_rrt_maps or (len(points) > 0 and isinstance(points[0], Node)):
    kwargs_without_marker = {k:v for k,v in kwargs.items() if k != 'marker'}
    if 'edgecolor' in kwargs:
      kwargs_without_marker['color'] = kwargs_without_marker.pop('edgecolor')
      # kwargs_without_marker.pop('edgecolor')
      kwargs.pop('edgecolor')
    for point in points:
      parent = None
      coord = None
      children = None
      if isinstance(point, Node):
        parent = point.parent
        coord = point.coord
      elif parents_map is not None:
        parent = parents_map.get(pt2tuple(point), None)
        coord = point
      if children_map is not None:
        children = children_map[pt2tuple(point)]
      if coord is not None and parent is not None:
        plot_line(ax, parent, coord, marker='', linestyle='-', linewidth=kwargs_without_marker.pop('linewidth', 2), **kwargs_without_marker)
      if coord is not None and children is not None and len(children) > 0:
        for child in children:
          plot_line(ax, coord, child, marker='', linestyle='-', linewidth=kwargs_without_marker.pop('linewidth', 2), **kwargs_without_marker)
  if 'edgecolor' in kwargs and len(points) > 0:
    print('edgecolor not handled!')
    print(len(points), points)
  for point in points:
    # pass
    plot_point(ax, point, **kwargs)
  return fig,ax

def plot_polygon(ax, poly, **kwargs):
  """
  Plots a Polygon to pyplot `ax`
  kwargs: facecolor='lightblue', edgecolor='red'
  """
  if hasattr(poly, 'is_empty') and poly.is_empty:
    print('empty polygon')
    return
  path = Path.make_compound_path(
    Path(np.asarray(poly.exterior.coords)[:, :2]),
    *[Path(np.asarray(ring.coords)[:, :2]) for ring in poly.interiors])

  patch = PathPatch(path, **kwargs)
  collection = PatchCollection([patch], **kwargs)
  
  ax.add_collection(collection, autolim=True)
  ax.autoscale_view()
  return collection

def plot_polygons(polygons: list = None, dims=None, **kwargs):
  if isinstance(polygons, Polygon):
    polygons = [polygons]
  if hasattr(polygons, 'is_empty') and polygons.is_empty:
    polygons = []
  if hasattr(polygons, 'geoms'):
    polygons = polygons.geoms
  if polygons is None:
    polygons = []
  ax = kwargs.pop('ax', None)
  if ax is None:
    fig,ax = plt.subplots()
  else:
    fig=None
  for polygon in polygons:
      plot_polygon(ax, polygon, **kwargs)
  #set aspect ratio to 1
  ratio = 1.0
  x_left, x_right = ax.get_xlim()
  y_low, y_high = ax.get_ylim()
  ax.set_aspect(abs((x_right-x_left)/(y_low-y_high))*ratio)
  return fig,ax

def plot_world(world: World, draw_obstacles: bool = True, **kwargs):
  ax = kwargs.pop('ax', None)
  if ax is None:
    fig,ax = plt.subplots()
  else:
    fig=None
  background_color = kwargs.pop('background_color', 'lightblue')
  border_color = kwargs.pop('border_color', 'red')
  obstacle_color = kwargs.pop('obstacle_color', 'black')
  obstacle_edge_color = kwargs.pop('obstacle_edge_color', 'red')
  plot_polygon(ax, world.getBoundingPolygon(), facecolor=background_color, edgecolor=border_color, **kwargs)
  if draw_obstacles:
    plot_polygons(world.obstacles.geoms, dims=world.dims, ax=ax, facecolor=obstacle_color, edgecolor=obstacle_edge_color, **kwargs)
  ax.set_xlim([0,world.dims[0]])
  ax.set_ylim([0,world.dims[1]])
  return fig,ax


def plot_potential_field(field, /,*, center=None, lim=None, figsize=(8,8), **kwargs):
    """
    Plot the vector field at a given center with bounds of [center-lim,center+lim] in both x,y directions
    """
    ax = kwargs.pop('ax', None)
    if ax is None:
      fig,ax = plt.subplots(figsize=figsize)
    else:
      fig=None
    
    field = normalize_field(field)
    dx,dy = (field[:,:,0], field[:,:,1])
    mags = np.linalg.norm(field, axis=2)

    shape = field.shape[:2]
    x, y = np.meshgrid(np.arange(shape[0],dtype=int), np.arange(shape[1],dtype=int))

    if center is None:
      center = (shape[0]//2, shape[1]//2)
    if lim is None:
      lim = min(shape[0], shape[1])//2

    ax.quiver(crop_field(x,center,lim), crop_field(y,center,lim), crop_field(dx,center,lim), crop_field(dy,center,lim), crop_field(mags,center,lim), angles='xy', scale_units='xy', scale=1, cmap=plt.cm.magma)

    return fig,ax


def plot_planner(world: World = None, curr_pos: Node = None, goal:Node = None, observations: BaseGeometry = None, position_history: list[Node] = None, rrt_tree:list[Node] = None, rrt_parents:dict = None, rrt_children:defaultdict[set] = None, planned_path:list[Node] = None, free_points:MultiPoint=None, extra_points:MultiPoint=None, potential_field:np.ndarray = None, **kwargs):
  if 'fig_ax' in kwargs:
    fig,ax = kwargs.pop('fig_ax')
  else:
    fig,ax = plt.subplots()
  draw_intersections = kwargs.pop('draw_intersections', None)

  if world is not None:
    plot_world(world, ax=ax, draw_obstacles=kwargs.pop('draw_world_obstacles', True), obstacle_color='black', obstacle_edge_color='red')
  if observations is not None:
    plot_polygons(observations, ax=ax, facecolor='green', edgecolor='blue')
  
  if potential_field is not None:
    plot_potential_field(potential_field, ax=ax)

  if draw_intersections is not None:
    if draw_intersections.is_empty:
      pass
    elif isinstance(draw_intersections, BaseMultipartGeometry):
      for intersection in draw_intersections.geoms:
        plot_points(list(intersection.coords), ax=ax, marker="o", markersize=10, markeredgecolor="purple", markerfacecolor="purple", linewidth=3)
    else:
      plot_points(list(draw_intersections.coords), ax=ax, marker="o", markersize=10, markeredgecolor="purple", markerfacecolor="purple", linewidth=3)
  
  # only draw RRT tree when there is no potential field being drawn
  if rrt_tree is not None and potential_field is None:
    plot_points(rrt_tree, parents_map=rrt_parents, children_map=rrt_children, ax=ax, marker=".", markersize=3, markeredgecolor="yellow", markerfacecolor="yellow", edgecolor='yellow', linewidth=1)
  
  if position_history is not None:
    plot_points(position_history, ax=ax, marker=".", markersize=3, markeredgecolor="pink", markerfacecolor="pink", edgecolor='pink', linewidth=1)
  if planned_path is not None:
    plot_points(planned_path, ax=ax, marker=".", markersize=5, markeredgecolor="orange", markerfacecolor="orange", edgecolor='orange', linewidth=1)
  if free_points is not None:
    plot_points(free_points, ax=ax, marker=".", markersize=5, markeredgecolor="blue", markerfacecolor="blue", linewidth=1)
  if extra_points is not None:
    if isinstance(extra_points, dict):
      for color,pts in extra_points.items():
        plot_points(pts, ax=ax, marker=".", markersize=8, markeredgecolor=color, markerfacecolor=color, linewidth=1)
    else:
      plot_points(extra_points, ax=ax, marker=".", markersize=8, markeredgecolor="white", markerfacecolor="white", linewidth=1)


  if curr_pos is not None:
    plot_point(ax, curr_pos, marker=".", markersize=6, markeredgecolor="red", markerfacecolor="red")
  if goal is not None:
    plot_point(ax, goal, marker=".", markersize=6, markeredgecolor="green", markerfacecolor="green")
  
  fig.set_size_inches(8,8)
  return fig,ax
