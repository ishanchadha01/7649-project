from abc import ABC, abstractmethod
from copy import deepcopy

from shapely.geometry.base import BaseGeometry
from shapely.geometry import Point

from farrt.node import Node
from farrt.world import World

class PartiallyObservablePlanner(ABC):
  def __init__(self, world: World, x_start: Node, x_goal: Node) -> None:
    self.world = world
    self.x_start = x_start
    self.x_goal = x_goal
    self.curr_pos = deepcopy(x_start)

  @abstractmethod
  def step(self) -> Point:
    pass

  @abstractmethod
  def run(self):
    pass