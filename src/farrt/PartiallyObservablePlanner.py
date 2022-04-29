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
<<<<<<< HEAD
=======
  def step(self) -> Point:
    pass

  @abstractmethod
>>>>>>> 4ecb18937c8c8f87c085004c0b528e216e694559
  def run(self):
    pass