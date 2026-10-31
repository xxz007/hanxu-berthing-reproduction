"""Quadtree for a filled occupancy grid, with indexed geometric queries.

Uniform free and blocked regions are both merged. Connectivity and collision
semantics match the existing reference: positive-length shared edges,
half-open point ownership, and closed blocked rectangles for LOS.
"""
import math

from .quadtree import Cell, QuadTree


def _segment_hits_box(start, end, box):
    enter, leave = 0.0, 1.0
    for p, q, lower, upper in (
            (start[0], end[0], box[0], box[2]),
            (start[1], end[1], box[1], box[3])):
        delta = q - p
        if delta == 0:
            if not lower <= p <= upper:
                return False
        else:
            lo, hi = sorted(((lower - p) / delta, (upper - p) / delta))
            enter, leave = max(enter, lo), min(leave, hi)
            if enter > leave:
                return False
    return True


class OccupancyQuadTree(QuadTree):
    def __init__(self, bounds, rows_bottom_to_top):
        self.bounds = tuple(float(v) for v in bounds)
        if len(self.bounds) != 4 or not all(math.isfinite(v) for v in self.bounds):
            raise ValueError('Map bounds must be four finite numbers')
        x0, y0, x1, y1 = self.bounds
        if x1 <= x0 or y1 <= y0:
            raise ValueError('Map bounds must have positive area')
        rows = rows_bottom_to_top
        n = len(rows)
        if n < 1 or n & (n - 1) or any(
                len(row) != n or any(v not in (0, 1) for v in row) for row in rows):
            raise ValueError('Occupancy must be a square power-of-two 0/1 grid')
        self.n = n
        self.max_depth = n.bit_length() - 1
        self.leaves = {}
        sums = [[0] * (n + 1)]
        for row in rows:
            current = [0]
            running = 0
            previous = sums[-1]
            for i, value in enumerate(row):
                running += int(value)
                current.append(previous[i + 1] + running)
            sums.append(current)

        def build(i0, j0, i1, j1, box, depth, code):
            count = sums[j1][i1] - sums[j0][i1] - sums[j1][i0] + sums[j0][i0]
            if count == 0 or count == (i1 - i0) * (j1 - j0):
                node = Cell(code, box, depth, count != 0)
                self.leaves[code] = node
                return node
            a, b, c, d = box
            ix, iy = (i0 + i1) // 2, (j0 + j1) // 2
            mx, my = (a + c) / 2, (b + d) / 2
            children = tuple(build(*indices, child_box, depth + 1, code + suffix)
                             for suffix, indices, child_box in (
                                 ('00', (i0, j0, ix, iy), (a, b, mx, my)),
                                 ('10', (ix, j0, i1, iy), (mx, b, c, my)),
                                 ('01', (i0, iy, ix, j1), (a, my, mx, d)),
                                 ('11', (ix, iy, i1, j1), (mx, my, c, d))))
            return Cell(code, box, depth, False, children)

        self.root = build(0, 0, n, n, self.bounds, 0, 'r')
        self.obstacles = tuple(node.bounds for node in self.leaves.values() if node.blocked)

    def neighbors(self, code):
        target = self.leaves[code]
        if target.blocked:
            return []
        a, b, c, d = target.bounds
        found = []
        stack = [self.root]
        while stack:
            node = stack.pop()
            l, s, r, t = node.bounds
            if node.code == code or c < l or r < a or d < s or t < b:
                continue
            if node.children:
                stack.extend(node.children)
            elif not node.blocked:
                vertical = (c == l or r == a) and max(b, s) < min(d, t)
                horizontal = (d == s or t == b) and max(a, l) < min(c, r)
                if vertical or horizontal:
                    found.append(node)
        return sorted(found, key=lambda cell: cell.center)

    def line_of_sight(self, start, end):
        self.locate(start)
        self.locate(end)
        stack = [self.root]
        while stack:
            node = stack.pop()
            if not _segment_hits_box(start, end, node.bounds):
                continue
            if node.children:
                stack.extend(node.children)
            elif node.blocked:
                return False
        return True
