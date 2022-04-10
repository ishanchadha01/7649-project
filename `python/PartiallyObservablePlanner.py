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
  def plan(self):
    pass

  @abstractmethod
  def step(self) -> Coord:
    pass

  def run(self):
    self.plan()
    while not self.world.reachedGoal(self.curr_pos, self.x_goal):
      self.curr_pos = self.step()
      found_new_obstacles = self.world.make_observations(self.curr_pos)
      if found_new_obstacles:
        self.plan()