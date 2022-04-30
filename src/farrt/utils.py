from typing import Any
from shapely.geometry import LineString, Point, MultiPoint, Polygon, MultiPolygon
from shapely.geometry.base import BaseGeometry

from farrt.node import Node

def as_point(pt: Any, /,*, error:bool = False) -> Point:
  if isinstance(pt, Point):
    return pt
  elif isinstance(pt, tuple):
    return Point(pt)
  elif isinstance(pt, list):
    return Point(pt)
  elif isinstance(pt, BaseGeometry):
    return Point(pt.centroid)
  elif isinstance(pt, Node):
    return pt.coord
  if error:
    raise TypeError('as_point() expects a Point, but got a {}'.format(type(pt)))
  return pt

def as_multipoint(mp: Any, /) -> MultiPoint:
  if isinstance(mp, list) or isinstance(mp, tuple):
    return MultiPoint(mp)
  if mp.is_empty:
    return MultiPoint()
  elif isinstance(mp, MultiPoint):
    return mp
  elif isinstance(mp, Point):
    return MultiPoint([mp])
  else:
    return MultiPoint([])

def as_multipolygon(mp: Any, /) -> MultiPolygon:
  if isinstance(mp, list) or isinstance(mp, tuple):
    return MultiPolygon(mp)
  if mp.is_empty:
    return MultiPolygon()
  elif isinstance(mp, MultiPolygon):
    return mp
  elif isinstance(mp, Polygon):
    return MultiPolygon([mp])
  else:
    return MultiPolygon([])

def pt2tuple(pt: Point, /) -> tuple[float,float]:
  return as_point(pt).coords[0]

def shapely_edge(pt0: Point, pt1: Point, /) -> LineString:
  return LineString([pt0, pt1])

def multipoint_without(mp: MultiPoint, pt: Point) -> MultiPoint:
  difference = mp - pt
  return as_multipoint(difference)
