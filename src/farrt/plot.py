import numpy as np
from matplotlib.path import Path
from matplotlib.patches import PathPatch
from matplotlib.collections import PatchCollection

from shapely.geometry import Polygon, Point, MultiPoint
from shapely.geometry.base import BaseGeometry, BaseMultipartGeometry
import matplotlib.pyplot as plt
from farrt.node import Node
from farrt.utils import as_point

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

def plot_points(points: list, parents_map:dict = None, ax=None, **kwargs):
  if ax is None:
    fig,ax = plt.subplots()
  else:
    fig=None
  if isinstance(points, MultiPoint):
    points = points.geoms
  if (parents_map is not None) or (len(points) > 0 and isinstance(points[0], Node)):
    kwargs_without_marker = {k:v for k,v in kwargs.items() if k != 'marker'}
    if 'edgecolor' in kwargs:
      kwargs_without_marker['color'] = kwargs_without_marker.pop('edgecolor')
      # kwargs_without_marker.pop('edgecolor')
      kwargs.pop('edgecolor')
    for point in points:
      parent = None
      coord = None
      if isinstance(point, Node):
        parent = point.parent
        coord = point.coord
      elif parents_map is not None:
        parent = parents_map.get(point.coords[0], None)
        coord = point
      if coord is not None and parent is not None:
        plot_line(ax, parent, coord, marker='', linestyle='-', linewidth=kwargs_without_marker.pop('linewidth', 2), **kwargs_without_marker)
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

def plot_planner(world: World = None, curr_pos: Node = None, goal:Node = None, observations: BaseGeometry = None, position_history: list[Node] = None, rrt_tree:list[Node] = None, rrt_parents:dict = None, planned_path:list[Node] = None, free_points:MultiPoint=None, **kwargs):
  if 'fig_ax' in kwargs:
    fig,ax = kwargs.pop('fig_ax')
  else:
    fig,ax = plt.subplots()
  draw_intersections = kwargs.pop('draw_intersections', None)

  if world is not None:
    plot_world(world, ax=ax, draw_obstacles=kwargs.pop('draw_world_obstacles', True), obstacle_color='black', obstacle_edge_color='red')
  if observations is not None:
    plot_polygons(observations, ax=ax, facecolor='green', edgecolor='blue')
  
  if draw_intersections is not None:
    if draw_intersections.is_empty:
      pass
    elif isinstance(draw_intersections, BaseMultipartGeometry):
      for intersection in draw_intersections.geoms:
        plot_points(list(intersection.coords), ax=ax, marker="o", markersize=10, markeredgecolor="purple", markerfacecolor="purple", linewidth=3)
    else:
      plot_points(list(draw_intersections.coords), ax=ax, marker="o", markersize=10, markeredgecolor="purple", markerfacecolor="purple", linewidth=3)
  
  if rrt_tree is not None:
    plot_points(rrt_tree, parents_map=rrt_parents, ax=ax, marker=".", markersize=3, markeredgecolor="yellow", markerfacecolor="yellow", edgecolor='yellow', linewidth=1)
  if position_history is not None:
    plot_points(position_history, ax=ax, marker=".", markersize=3, markeredgecolor="pink", markerfacecolor="pink", edgecolor='pink', linewidth=1)
  if planned_path is not None:
    plot_points(planned_path, ax=ax, marker=".", markersize=5, markeredgecolor="orange", markerfacecolor="orange", edgecolor='orange', linewidth=1)
  if free_points is not None:
    plot_points(free_points, ax=ax, marker=".", markersize=5, markeredgecolor="blue", markerfacecolor="blue", linewidth=1)

  if curr_pos is not None:
    plot_point(ax, curr_pos, marker=".", markersize=6, markeredgecolor="red", markerfacecolor="red")
  if goal is not None:
    plot_point(ax, goal, marker=".", markersize=6, markeredgecolor="green", markerfacecolor="green")
  
  fig.set_size_inches(8,8)
  return fig,ax