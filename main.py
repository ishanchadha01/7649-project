import os
import shutil
from matplotlib import pyplot as plt
from farrt.PartiallyObservablePlanner import PartiallyObservablePlanner

from farrt.planners import RRTStar, FARRTStar
from farrt.plot import plot_point, plot_world
from farrt.world import World

class Comparison():
  def __init__(self, planners, world, start, goal, **kwargs):
    self.world = world
    self.outdir = kwargs.pop('outdir', 'comparison-gifs')
    self.planner_name_str = kwargs.pop('planner_name_str', '-'.join([Planner.__name__.lower() for Planner in planners]))
    self.run_count = kwargs.pop('run_count', Comparison.default_run_count(self.outdir, self.planner_name_str, planners))

    self.gui = kwargs.get('gui', True)

    self.planner_outdir = os.path.join(self.outdir, f'{self.planner_name_str}-{self.run_count}')
    self.planners: list[PartiallyObservablePlanner] = [Planner(world, start, goal, outdir=self.planner_outdir, gif_name=f'{Planner.__name__.lower()}', **kwargs) for Planner in planners]

  @staticmethod
  def default_run_count(outdir: str, prefix:str, planners) -> int:
    count = 0
    while os.path.exists(os.path.join(outdir, f'{prefix}-{count}')) and all([os.path.exists(os.path.join(outdir, f'{prefix}-{count}', f'{Planner.__name__.lower()}.gif')) for Planner in planners]):
      count += 1
    return count

  def run(self):
    if self.gui:
      fig,ax = (None,None)#plt.subplots()
      if os.path.exists(self.planner_outdir):
        shutil.rmtree(self.planner_outdir)
      os.makedirs(self.planner_outdir)
    for planner in self.planners:
      print(f'Running {planner.__class__.__name__}')
      planner.run()
      print(f'Finished {planner.__class__.__name__}')
      planner.render(visualize=True)
    
if __name__ == '__main__':
  world = World()
  fig,ax = plot_world(world)
  start = world.random_position(not_blocked=True)
  goal = world.random_position(not_blocked=True)
  plot_point(ax, start, marker=".", markersize=6, markeredgecolor="red", markerfacecolor="red")
  plot_point(ax, goal, marker=".", markersize=6, markeredgecolor="green", markerfacecolor="green")
  plt.show()

  comp = Comparison([FARRTStar, RRTStar], world, start, goal, gui=True, force_no_visualization=True)
  comp.run()