from shapely.geometry import LineString, Point, MultiPoint, Polygon, MultiPolygon
from shapely.geometry.base import BaseGeometry

from farrt.node import Node

def as_point(pt: Point, *, error:bool = False) -> Point:
  if isinstance(pt, Node):
    pt = pt.coord
  if isinstance(pt, BaseGeometry):
    pt = pt.centroid
  if isinstance(pt, tuple):
    pt = Point(pt)
  if not isinstance(pt, Point) and error:
    raise TypeError('as_point() expects a Point, but got a {}'.format(type(pt)))
  return pt

def as_multipoint(mp: MultiPoint) -> MultiPoint:
  if mp.is_empty:
    return MultiPoint()
  elif isinstance(mp, MultiPoint):
    return mp
  elif isinstance(mp, Point):
    return MultiPoint([mp])
  else:
    return MultiPoint([])

def as_multipolygon(mp: MultiPolygon) -> MultiPolygon:
  if mp.is_empty:
    return MultiPolygon()
  elif isinstance(mp, MultiPolygon):
    return mp
  elif isinstance(mp, Polygon):
    return MultiPolygon([mp])
  else:
    return MultiPolygon([])

def shapely_edge(pt0: Point, pt1: Point) -> LineString:
  return LineString([pt0, pt1])

def multipoint_without(mp: MultiPoint, pt: Point) -> MultiPoint:
  difference = mp - pt
  return as_multipoint(difference)
