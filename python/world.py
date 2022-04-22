from abc import ABC, abstractmethod
from typing import TypeVar

Coord = TypeVar('Coord', tuple)

class World(ABC):
  def __init__(self):
    self.dims = [90, 90]
    self.obstacles = set()

  @abstractmethod
  def reached_goal(position: Coord, goal: Coord) -> bool:
    pass

  @abstractmethod
  def make_observations(position: Coord) -> bool:
    pass

  @abstractmethod
  def random_position(self) -> Coord:
    pass

  @abstractmethod
  def obstacle_detected(self) -> None:
    pass

  @abstractmethod
  def obstacle_vanished(self) -> None:
    pass