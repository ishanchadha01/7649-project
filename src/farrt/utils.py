from shapely.geometry import LineString, Point, MultiPoint

def shapely_edge(pt0: Point, pt1: Point) -> LineString:
  return LineString([pt0, pt1])

def multipoint_without(mp: MultiPoint, pt: Point) -> MultiPoint:
  difference = mp - pt
  if isinstance(difference, MultiPoint):
    return difference
  elif isinstance(difference, Point):
    return MultiPoint([difference])
  else:
    return MultiPoint([])
