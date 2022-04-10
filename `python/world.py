from abc import ABC, abstractmethod
from typing import TypeVar

Coord = TypeVar('Coord', tuple)

class World(ABC):
  def __init__():
    pass

  @abstractmethod
  def reachedGoal(position: Coord, goal: Coord) -> bool:
    pass

  @abstractmethod
  def make_observations(position: Coord) -> bool:
    pass

  @abstractmethod
  def random_position(self) -> Coord:
    pass