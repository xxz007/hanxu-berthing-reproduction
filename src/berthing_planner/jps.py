"""Online JPS on uniform 8-neighbour grids without corner cutting.

Independent implementation of Hu et al. (2021), section 4.1, Fig.1 and
definitions 1-3. No JPS+, goal bounding, preprocessing or author code claim.
"""
import heapq
import math

DIRECTIONS=((-1,-1),(-1,0),(-1,1),(0,-1),(0,1),(1,-1),(1,0),(1,1))


class SquareGrid:
    def __init__(self,rows,cell_size=1.0,origin=(0.0,0.0)):
        if not rows or not rows[0] or any(len(row)!=len(rows[0]) or any(v not in (0,1) for v in row) for row in rows):
            raise ValueError('Expected a nonempty rectangular 0/1 occupancy grid')
        if not math.isfinite(cell_size) or cell_size<=0 or len(origin)!=2 or not all(math.isfinite(v) for v in origin):
            raise ValueError('Expected positive finite cell size and finite origin')
        self.rows=[bytes(row) for row in rows]
        self.nx,self.ny=len(rows[0]),len(rows)
        self.cell_size=cell_size
        self.origin=tuple(origin)

    @classmethod
    def from_rectangles(cls,bounds,obstacles,cell_size):
        x0,y0,x1,y1=bounds
        if not math.isfinite(cell_size) or cell_size<=0:
            raise ValueError('Cell size must be finite and positive')
        nx,ny=math.ceil((x1-x0)/cell_size),math.ceil((y1-y0)/cell_size)
        rows=[bytearray(nx) for _ in range(ny)]
        # Partial boundary cells extend outside the source map: block them.
        for j in range(ny):
            for i in range(nx):
                if x0+(i+1)*cell_size>x1 or y0+(j+1)*cell_size>y1:
                    rows[j][i]=1
        for l,b,r,t in obstacles:
            for j in range(max(0,math.floor((b-y0)/cell_size)),min(ny,math.ceil((t-y0)/cell_size))):
                for i in range(max(0,math.floor((l-x0)/cell_size)),min(nx,math.ceil((r-x0)/cell_size))):
                    rows[j][i]=1
        return cls(rows,cell_size,(x0,y0))

    def free(self,x,y):
        return 0<=x<self.nx and 0<=y<self.ny and not self.rows[y][x]

    def can_step(self,x,y,dx,dy):
        return self.free(x+dx,y+dy) and (not dx or not dy or
                    self.free(x+dx,y) and self.free(x,y+dy))

    def locate(self,point):
        x,y=(math.floor((point[j]-self.origin[j])/self.cell_size) for j in (0,1))
        if not self.free(x,y):
            raise ValueError('Endpoint is outside or blocked in the uniform grid')
        return x,y

    def center(self,node):
        return tuple(self.origin[j]+(node[j]+.5)*self.cell_size for j in (0,1))


def forced_directions(grid,x,y,dx,dy):
    """Cardinal arrival: side accessible here, inaccessible one cell behind."""
    result=[]
    for sx,sy in ((-dy,dx),(dy,-dx)):
        if grid.free(x+sx,y+sy) and not grid.free(x-dx+sx,y-dy+sy):
            result.extend(((sx,sy),(dx+sx,dy+sy)))
    return result


def straight_jump(grid,x,y,dx,dy,goal,stats):
    while grid.can_step(x,y,dx,dy):
        x,y=x+dx,y+dy
        stats['jump_scan_steps']+=1
        if (x,y)==goal or forced_directions(grid,x,y,dx,dy):
            return x,y
    return None


def jump(grid,x,y,dx,dy,goal,stats):
    if not dx or not dy:
        return straight_jump(grid,x,y,dx,dy,goal,stats)
    while grid.can_step(x,y,dx,dy):
        x,y=x+dx,y+dy
        stats['jump_scan_steps']+=1
        if (x,y)==goal:
            return x,y
        if (straight_jump(grid,x,y,dx,0,goal,stats) is not None or
            straight_jump(grid,x,y,0,dy,goal,stats) is not None):
            return x,y
    return None


def search_grid(grid,start,goal,algorithm='jps'):
    """Return macro path and counts; caller times search excluding validation.

    opened_unique counts cells ever queued, closed_unique counts cells expanded
    including the target. jump_scan_steps includes repeated scans, not unique
    cells. A* uses exactly the same step rules, costs, heuristic and heap ties.
    """
    if algorithm not in ('jps','astar'):
        raise ValueError('Expected jps or astar')
    start,goal=tuple(start),tuple(goal)
    if not grid.free(*start) or not grid.free(*goal):
        raise ValueError('Search endpoints must be free grid cells')
    def heuristic(node):
        dx,dy=abs(node[0]-goal[0]),abs(node[1]-goal[1])
        return grid.cell_size*(max(dx,dy)+(math.sqrt(2)-1)*min(dx,dy))
    costs={start:0.0};parents={start:start};closed=set()
    queue=[(heuristic(start),0.0,start)]
    stats={'opened_unique':1,'closed_unique':0,'jump_scan_steps':0,'neighbor_directions_tested':0}
    while queue:
        _,cost,node=heapq.heappop(queue)
        if node in closed or cost!=costs[node]:
            continue
        closed.add(node)
        if node==goal:
            path=[node]
            while path[-1]!=start:
                path.append(parents[path[-1]])
            stats.update(opened_unique=len(costs),closed_unique=len(closed))
            return {'status':'path_found','cell_path':list(reversed(path)),
                    'grid_length_m':cost,**stats}
        directions=DIRECTIONS
        if algorithm=='jps' and node!=start:
            p=parents[node]
            dx=(node[0]>p[0])-(node[0]<p[0]);dy=(node[1]>p[1])-(node[1]<p[1])
            directions=((dx,dy),(dx,0),(0,dy)) if dx and dy else [(dx,dy),*forced_directions(grid,*node,dx,dy)]
        for dx,dy in directions:
            stats['neighbor_directions_tested']+=1
            if not grid.can_step(*node,dx,dy):
                continue
            nxt=jump(grid,*node,dx,dy,goal,stats) if algorithm=='jps' else (node[0]+dx,node[1]+dy)
            if nxt is None or nxt in closed:
                continue
            candidate=cost+grid.cell_size*math.dist(node,nxt)
            if candidate<costs.get(nxt,math.inf):
                costs[nxt]=candidate;parents[nxt]=node
                heapq.heappush(queue,(candidate+heuristic(nxt),candidate,nxt))
    stats.update(opened_unique=len(costs),closed_unique=len(closed))
    return {'status':'no_path','cell_path':[],'grid_length_m':None,**stats}
