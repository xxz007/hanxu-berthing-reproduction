"""Euclidean A* and an explicit Theta*-style LOS parent relaxation.

The latter is a reproducible interpretation, not recovered author code.
"""
import heapq
import math
import time


def astar(tree, start, goal, any_angle=False):
    start, goal = tuple(start), tuple(goal)
    s, g = tree.locate(start), tree.locate(goal)
    if s.blocked or g.blocked:
        raise ValueError('Start or goal is in a blocked planning cell')
    if not tree.line_of_sight(start, start) or not tree.line_of_sight(goal, goal):
        raise ValueError('Endpoint touches a blocked planning cell')
    points = {c.code: c.center for c in tree.leaves.values() if not c.blocked}
    points.update(start=start, goal=goal)
    adjacency = {}
    opened, closed, parent, costs = {'start'}, set(), {'start': 'start'}, {'start': 0.0}
    queue = [(math.dist(start, goal), 0.0, 'start')]
    begin = time.perf_counter()
    while queue:
        _, old_cost, u = heapq.heappop(queue)
        if old_cost != costs[u] or u in closed:
            continue
        closed.add(u)
        if u == 'goal':
            chain = [u]
            while chain[-1] != 'start':
                chain.append(parent[chain[-1]])
            path = [points[k] for k in reversed(chain)]
            path = [p for i, p in enumerate(path) if i == 0 or p != path[i-1]]
            return dict(path=path, length_m=costs[u], opened=len(opened), closed=len(closed),
                        search_seconds=time.perf_counter()-begin,
                        closed_points=[points[k] for k in sorted(closed)])
        if u == 'start':
            neighbors = [s.code]
            if s.code == g.code:
                neighbors.append('goal')
        else:
            if u not in adjacency:
                adjacency[u] = [c.code for c in tree.neighbors(u)]
            neighbors = list(adjacency[u])
            if u == g.code:
                neighbors.append('goal')
        for v in neighbors:
            if v in closed or not tree.line_of_sight(points[u], points[v]):
                continue
            p = u
            candidate = costs[u] + math.dist(points[u], points[v])
            if any_angle and tree.line_of_sight(points[parent[u]], points[v]):
                alternate = costs[parent[u]] + math.dist(points[parent[u]], points[v])
                if alternate <= candidate:
                    p, candidate = parent[u], alternate
            if candidate < costs.get(v, math.inf):
                costs[v], parent[v] = candidate, p
                opened.add(v)
                heapq.heappush(queue, (candidate+math.dist(points[v], goal), candidate, v))
    raise ValueError('No path on this planning graph')


def shortcut(tree, path):
    """Explicit engineering pass, separate from the search itself."""
    result, i = [path[0]], 0
    while i < len(path)-1:
        j = len(path)-1
        while j > i+1 and not tree.line_of_sight(path[i], path[j]):
            j -= 1
        result.append(path[j])
        i = j
    return result
