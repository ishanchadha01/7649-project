class Node:
    def __init__(self, coord=tuple, parent=None) -> None:
        self.parent = parent
        self.coord = coord
        self.children = []
