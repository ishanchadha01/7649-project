from abc import ABC, abstractmethod
from typing import List, TypeVar
import random
# import numpy as np
from shapely.geometry import Point, MultiPolygon, Polygon, CAP_STYLE
from shapely.geometry.base import BaseGeometry

from farrt.node import Node


#class World(ABC):
class World():

  def __init__(self, dims: List[float] = None, obstacles: BaseGeometry = None) -> None:
    self.dims = dims or [100,100]
    self.obstacles: BaseGeometry = obstacles or World.generate_default_obstacles(self.dims)

  @classmethod
  def generate_default_obstacles(cls, dims: List[float], num_obstacles=50) -> BaseGeometry:
    obstacles = BaseGeometry()
    for i in range(num_obstacles):
      coord = Point(*[random.random() * dim for dim in dims])
      size = random.randint(2,4)
      poly = coord.buffer(size, cap_style=CAP_STYLE.square)
      obstacles = obstacles.union(poly)
    return obstacles.intersection(Polygon([[0,0], [0, dims[1]], [dims[0], dims[1]], [dims[0], 0], [0,0]]))

  def getBoundingPoly(self) -> Polygon:
    return Polygon([[0,0], [0, self.dims[1]], [self.dims[0], self.dims[1]], [self.dims[0], 0], [0,0]])

  #@abstractmethod
  def reached_goal(position: Point, goal: Point) -> bool:
    pass

  #@abstractmethod
  def make_observations(self, position: Point|Node, radius: float) -> BaseGeometry:
    if isinstance(position, Node):
      position = position.coord
    circle = position.buffer(radius)
    obstervation = self.obstacles.intersection(circle)
    if isinstance(obstervation, MultiPolygon):
      return obstervation
    elif isinstance(obstervation, Polygon):
      return MultiPolygon([obstervation])

  #@abstractmethod
  def random_position(self) -> Point:
    out = Point(*[random.random() * dim for dim in self.dims])
    # print(out)
    return out

  # #@abstractmethod
  # def obstacle_detected(self) -> None:
  #   pass

  # #@abstractmethod
  # def obstacle_vanished(self) -> None:
  #   pass