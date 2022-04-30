from abc import ABC, abstractmethod
from typing import List, TypeVar
import random
from shapely.geometry import Point, MultiPolygon, Polygon, CAP_STYLE

from farrt.node import Node
from farrt.utils import as_multipolygon, as_point


#class World(ABC):
class World():
  def __init__(self, dims: List[float] = None, obstacles: MultiPolygon = None) -> None:
    self.dims = dims or [100,100]
    self.obstacles: MultiPolygon = obstacles or World.generate_default_obstacles(self.dims)

  @staticmethod
  def generate_default_obstacles(dims: List[float], num_obstacles=100) -> MultiPolygon:
    obstacles = MultiPolygon()
    for i in range(num_obstacles):
      coord = Point(*[random.random() * dim for dim in dims])
      size = 2 + random.random() * 4
      poly = coord.buffer(size, cap_style=CAP_STYLE.square)
      obstacles = obstacles.union(poly)
    return obstacles.intersection(Polygon([[0,0], [0, dims[1]], [dims[0], dims[1]], [dims[0], 0], [0,0]]))

  def getBoundingPolygon(self) -> Polygon:
    return Polygon([[0,0], [0, self.dims[1]], [self.dims[0], self.dims[1]], [self.dims[0], 0], [0,0]])

  def getBounds(self) -> tuple[float,float,float,float]:
    return self.getBoundingPolygon().bounds

  #@abstractmethod
  def make_observations(self, position: Point|Node, radius: float) -> MultiPolygon:
    position = as_point(position)
    circle = position.buffer(radius)
    obstervation = self.obstacles.intersection(circle)
    return as_multipolygon(obstervation)

  #@abstractmethod
  def random_position(self, /,*, not_blocked=False) -> Point:
    out = Point(*[random.random() * dim for dim in self.dims])
    if not_blocked:
      while self.obstacles.intersects(out.buffer(3)):
        out = Point(*[random.random() * dim for dim in self.dims])
    return out
