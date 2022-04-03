from node import Node


def rrt(x_start, x_goal, iters=1000):
    tree = [x_start]
    for _ in range(iters):
        x_rand = sample(x_goal)
        x_nearest = nearest(tree, x_rand)
        x_new = steer(x_nearest, x_rand)
        tree.append(x_new)
        if is_valid_edge(x_nearest, x_new):
            tree.append(x_new)
            x_new.parent = x_nearest
            x_nearest.children.append(x_new)
    path = [x_goal]
    curr = x_goal
    while curr != x_start:
        path.append(curr.parent)
        curr = curr.parent
    return path


def sample(goal_node):
    pass


def nearest(tree, node2):
    pass


def steer(node1, node2):
    pass


def is_valid_edge(node1, node2):
    pass