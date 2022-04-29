import numpy as np
from matplotlib.path import Path
from matplotlib.patches import PathPatch
from matplotlib.collections import PatchCollection

from shapely.geometry import Polygon
import matplotlib.pyplot as plt

from farrt.world import World


# Plots a Polygon to pyplot `ax`
def plot_polygon(ax, poly, **kwargs):
  path = Path.make_compound_path(
    Path(np.asarray(poly.exterior.coords)[:, :2]),
    *[Path(np.asarray(ring.coords)[:, :2]) for ring in poly.interiors])

  patch = PathPatch(path, **kwargs)
  collection = PatchCollection([patch], **kwargs)
  
  ax.add_collection(collection, autolim=True)
  ax.autoscale_view()
  return collection

def plot_polygons(polygons: list = None, dims=None, **kwargs):
  if polygons is None:
    polygons = []
  elif hasattr(polygons, 'is_empty') and polygons.is_empty:
    polygons = []
  elif hasattr(polygons, 'geoms'):
    polygons = polygons.geoms
  fig,ax = plt.subplots()
  for polygon in polygons:
      plot_polygon(ax, polygon, **kwargs)
  if dims is not None:
    ax.set_xlim([0,dims[0]])
    ax.set_ylim([0,dims[1]])
  #set aspect ratio to 1
  ratio = 1.0
  x_left, x_right = ax.get_xlim()
  y_low, y_high = ax.get_ylim()
  ax.set_aspect(abs((x_right-x_left)/(y_low-y_high))*ratio)
  return fig,ax

def plot_world(world: World = None, **kwargs):
  if world is None:
    world = World()
  return plot_polygons(world.obstacles.geoms, dims=world.dims, **kwargs)



# # Input polygon with two holes
# # (remember exterior point order is ccw, holes cw else
# # holes may not appear as holes.)
# polygon = Polygon(shell=((0,0),(10,0),(10,10),(0,10)),
#                   holes=(((1,3),(5,3),(5,1),(1,1)),
#                          ((9,9),(9,8),(8,8),(8,9))))

# fig, ax = plt.subplots()