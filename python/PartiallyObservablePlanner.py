from abc import ABC, abstractmethod
from typing import Generic, TypeVar
from main.node import Node
from main.world import World, Coord

class PartiallyObservablePlanner(ABC):
  def __init__(self, world: World, x_start: Coord, x_goal: Coord) -> None:
    self.world = world
    self.x_start = x_start
    self.x_goal = x_goal
    self.curr_pos = x_start

  @abstractmethod
  def step(self) -> Coord:
    pass

  @abstractmethod
  def run(self):
    pass