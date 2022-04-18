class Node:
    def __init__(self, coord: tuple, parent: 'Node'=None) -> None:
        self.parent = parent
        self.coord = coord
        self.children = []
    
    def coord_copy(self):
        return Node(self.coord)
