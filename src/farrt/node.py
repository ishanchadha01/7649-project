from farrt.world import Coord

class Node:
  def __init__(self, coord: Coord, parent: 'Node'=None) -> None:
    self.parent = parent
    self.coord = coord
    self.children = []
  
  def coord_copy(self):
    return Node(self.coord)
