from shapely.geometry import LineString, Point
from shapely.geometry.base import BaseGeometry

class Node:
  def __init__(self, coord: Point, parent: 'Node'=None) -> None:
    if isinstance(coord, Point):
      self.coord = coord
    else:
      raise TypeError('Node() expects a Point, but got a {}'.format(type(coord)))
    self.parent = parent
    self.children = []
    self.lmc = float('inf')

  def __deepcopy__(self, memo):
    return Node(Point(self.coord.x, self.coord.y), self.parent)
  
  def same_as(self, other: 'Node') -> bool:
    return self.coord == other.coord

  def dist(self, other: 'Node') -> float:
    return self.coord.distance(other.coord)
  
  def edgeTo(self, other: 'Node') -> LineString:
    return LineString([self.coord, other.coord])
