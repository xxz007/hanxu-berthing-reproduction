"""Validated reconstructed occupancy -> the same quadtree/query interface.

Pure stdlib runtime; preprocessing tools are separate. Obstacles are closed
blocked leaf rectangles. Source chart axes and uncalibrated heading are metadata.
"""
import math
from .quadtree import QuadTree,Cell


class RasterQuadTree(QuadTree):
    def __init__(self,data):
        if data.get('schema_version')!=1 or data.get('status')!='reconstructed_centerline_not_author_raw_map':
            raise ValueError('Expected a recorded R1 centerline reconstruction')
        self.bounds=tuple(data['bounds'])
        if len(self.bounds)!=4 or not all(math.isfinite(v) for v in self.bounds):
            raise ValueError('Map bounds must be four finite numbers')
        x0,y0,x1,y1=self.bounds
        if x1<=x0 or y1<=y0:
            raise ValueError('Map bounds must have positive area')
        n=data['n']
        rows=data['rows_bottom_to_top']
        if type(n) is not int or n<2 or n&(n-1) or len(rows)!=n or any(len(row)!=n or any(v not in (0,1) for v in row) for row in rows):
            raise ValueError('Occupancy must be a square power-of-two 0/1 grid')
        self.max_depth=n.bit_length()-1
        self.leaves={}
        def build(i0,j0,i1,j1,depth,code):
            occupied=any(rows[j][i] for j in range(j0,j1) for i in range(i0,i1))
            bounds=(x0+i0*(x1-x0)/n,y0+j0*(y1-y0)/n,x0+i1*(x1-x0)/n,y0+j1*(y1-y0)/n)
            if not occupied or i1-i0==1:
                cell=Cell(code,bounds,depth,occupied)
                self.leaves[code]=cell
                return cell
            mx,my=(i0+i1)//2,(j0+j1)//2
            children=tuple(build(*box,depth+1,code+suffix) for suffix,box in
                [('00',(i0,j0,mx,my)),('10',(mx,j0,i1,my)),('01',(i0,my,mx,j1)),('11',(mx,my,i1,j1))])
            return Cell(code,bounds,depth,False,children)
        self.root=build(0,0,n,n,0,'r')
        self.obstacles=tuple(c.bounds for c in self.leaves.values() if c.blocked)
