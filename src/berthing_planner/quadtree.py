"""Teaching quadtree: half-open ownership, edge adjacency, closed-box LOS."""

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class Cell:
    code: str
    bounds: tuple[float, float, float, float]
    depth: int
    blocked: bool
    children: tuple["Cell", ...] = ()

    @property
    def center(self):
        x0, y0, x1, y1 = self.bounds
        return ((x0 + x1) / 2, (y0 + y1) / 2)


class QuadTree:
    def __init__(self, bounds, obstacles, max_depth):
        self.bounds = tuple(float(v) for v in bounds)
        self.obstacles = tuple(tuple(float(v) for v in box) for box in obstacles)
        for box in (self.bounds, *self.obstacles):
            if len(box) != 4 or not all(math.isfinite(v) for v in box):
                raise ValueError("Bounds must contain four finite coordinates")
            if box[0] >= box[2] or box[1] >= box[3]:
                raise ValueError("Rectangles must have positive width and height")
        x0, y0, x1, y1 = self.bounds
        for a, b, c, d in self.obstacles:
            if not (x0 <= a < c <= x1 and y0 <= b < d <= y1):
                raise ValueError("Obstacles must lie inside the map")
        if type(max_depth) is not int or max_depth < 0:
            raise ValueError("max_depth must be a nonnegative integer")
        self.max_depth = max_depth
        self.leaves = {}

        def build(box, depth, code):
            a, b, c, d = box
            occupied = any(a < r and l < c and b < t and s < d
                           for l, s, r, t in self.obstacles)
            if not occupied or depth == max_depth:
                leaf = Cell(code, box, depth, occupied)
                self.leaves[code] = leaf
                return leaf
            mx, my = (a + c) / 2, (b + d) / 2
            quadrants = (
                ("00", (a, b, mx, my)), ("10", (mx, b, c, my)),
                ("01", (a, my, mx, d)), ("11", (mx, my, c, d)),
            )
            children = tuple(build(region, depth + 1, code + suffix)
                             for suffix, region in quadrants)
            return Cell(code, box, depth, False, children)

        self.root = build(self.bounds, 0, "r")

    def locate(self, point):
        """Descend the tree. Left/bottom edges belong to a cell; right/top do not."""
        x, y = point
        x0, y0, x1, y1 = self.bounds
        if not (math.isfinite(x) and math.isfinite(y) and x0 <= x < x1 and y0 <= y < y1):
            raise ValueError("Point must be finite and inside the half-open map")
        node = self.root
        while node.children:
            a, b, c, d = node.bounds
            index = int(x >= (a + c) / 2) + 2 * int(y >= (b + d) / 2)
            node = node.children[index]
        return node

    def neighbors(self, code):
        """Geometric reference: scan free leaves sharing an edge of positive length."""
        node = self.leaves[code]
        if node.blocked:
            return []
        a, b, c, d = node.bounds
        result = []
        for other in self.leaves.values():
            if other.code == code or other.blocked:
                continue
            l, s, r, t = other.bounds
            vertical = (c == l or r == a) and max(b, s) < min(d, t)
            horizontal = (d == s or t == b) and max(a, l) < min(c, r)
            if vertical or horizontal:
                result.append(other)
        return sorted(result, key=lambda cell: cell.center)

    def line_of_sight(self, start, end):
        """Segment/closed-AABB intersection, including touching and zero-length cases.

        This is a geometric reference, not the thesis's sampled LOS implementation.
        At a coarse depth, mixed leaves are conservatively blocked in full.
        """
        self.locate(start)
        self.locate(end)
        for leaf in self.leaves.values():
            if not leaf.blocked:
                continue
            x0, y0, x1, y1 = leaf.bounds
            enter, leave = 0.0, 1.0
            for p, q, lower, upper in (
                (start[0], end[0], x0, x1), (start[1], end[1], y0, y1),
            ):
                delta = q - p
                if delta == 0:
                    if p < lower or p > upper:
                        break
                else:
                    t0, t1 = sorted(((lower - p) / delta, (upper - p) / delta))
                    enter, leave = max(enter, t0), min(leave, t1)
                    if enter > leave:
                        break
            else:
                return False
        return True
