from typing import Any
import numpy as np
from shapely.geometry import LineString, Point, MultiPoint, Polygon, MultiPolygon
from shapely.geometry.base import BaseGeometry

from farrt.node import Node

def as_point(pt: Any, /,*, error:bool = False) -> Point:
  if isinstance(pt, Point):
    return pt
  if isinstance(pt, tuple):
    return Point(pt)
  if isinstance(pt, list):
    return Point(pt)
  if isinstance(pt, BaseGeometry):
    return Point(pt.centroid)
  if isinstance(pt, Node):
    return pt.coord
  if error:
    raise TypeError('as_point() expects a Point, but got a {}'.format(type(pt)))
  return pt

def as_multipoint(mp: Any, /) -> MultiPoint:
  if isinstance(mp, MultiPoint):
    return mp
  if isinstance(mp, list) or isinstance(mp, tuple):
    return MultiPoint(mp)
  if isinstance(mp, set):
    return MultiPoint(list(mp))
  if mp.is_empty:
    return MultiPoint()
  if isinstance(mp, Point):
    return MultiPoint([mp])
  else:
    return MultiPoint([])

def as_multipolygon(mp: Any, /) -> MultiPolygon:
  if isinstance(mp, list) or isinstance(mp, tuple):
    return MultiPolygon(mp)
  if mp.is_empty:
    return MultiPolygon()
  if isinstance(mp, MultiPolygon):
    return mp
  if isinstance(mp, Polygon):
    return MultiPolygon([mp])
  else:
    return MultiPolygon([])

def as_linestring(edge: Any, /,*, error:bool = False) -> LineString:
  if isinstance(edge, LineString):
    return edge
  if isinstance(edge, tuple):
    return LineString(edge)
  if isinstance(edge, list):
    return LineString(edge)
  if error:
    raise TypeError('as_edge() expects a LineString, but got a {}'.format(type(edge)))
  return edge

def pt2tuple(pt: Point, /) -> tuple[float,float]:
  return as_point(pt).coords[0]

def shapely_edge(pt0: Point, pt1: Point, /) -> LineString:
  return LineString([pt0, pt1])

def line2tuple(line: LineString, /) -> tuple[tuple[float,float],tuple[float,float]]:
  return tuple(as_linestring(line).coords)

def multipoint_without(mp: MultiPoint, pt: Point) -> MultiPoint:
  difference = as_multipoint(mp) - pt
  return as_multipoint(difference)


def crop_field(data, center, lim):
    ''' 
    Crop the image by selecting a center point and only including
    all pixels within lim distance of the center point
    '''
    return data[center[1]-lim:center[1]+lim, center[0]-lim:center[0]+lim]

def normalize_field(field):
    '''
    Max-normalize the field based on magnitude of vec
    '''
    # divide each vector by max of magnitude of any vector
    magnitude = np.linalg.norm(field, axis=2)
    field = field / np.max(magnitude)

    return field
