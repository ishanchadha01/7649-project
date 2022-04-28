from abc import ABC, abstractmethod
from copy import deepcopy
from typing import Generic, TypeVar
from node import Node
from world import World, Coord

class PartiallyObservablePlanner(ABC):
  def __init__(self, world: World, x_start: Node, x_goal: Node) -> None:
    self.world = world
    self.x_start = x_start
    self.x_goal = x_goal
    self.curr_pos = deepcopy(x_start)
    self.detected_edges = []

  @abstractmethod
  def step(self) -> Coord:
    pass

  @abstractmethod
  def run(self):
    pass