"""Synthetic harbor integration. Every substitution is explicit in the config."""
from dataclasses import replace
import math
import time
from .quadtree import QuadTree,Cell
from .search import astar,shortcut
from .curves import g2_smooth, sample_path, collision_free, wrap, curvature_maximum
from .reeds_shepp import connect


class UniformGrid(QuadTree):
    def __init__(self,bounds,obstacles,max_depth):
        # Reuse validation and closed-leaf LOS semantics at exactly the same depth.
        super().__init__(bounds,obstacles,max_depth)
        self.n = 2**max_depth
        x0,y0,x1,y1 = self.bounds
        self.dx,self.dy = (x1-x0)/self.n,(y1-y0)/self.n
        self.leaves = {}
        for i in range(self.n):
            for j in range(self.n):
                box = (x0+i*self.dx,y0+j*self.dy,x0+(i+1)*self.dx,y0+(j+1)*self.dy)
                blocked = any(box[0]<r and l<box[2] and box[1]<t and b<box[3] for l,b,r,t in obstacles)
                key = f'{i},{j}'
                self.leaves[key] = Cell(key,box,max_depth,blocked)

    def locate(self,point):
        x0,y0,x1,y1 = self.bounds
        if not (x0<=point[0]<x1 and y0<=point[1]<y1):
            raise ValueError('Point outside map')
        return self.leaves[f'{int((point[0]-x0)/self.dx)},{int((point[1]-y0)/self.dy)}']

    def neighbors(self,code):
        i,j = map(int,code.split(','))
        return [self.leaves[f'{a},{b}'] for a,b in ((i-1,j),(i+1,j),(i,j-1),(i,j+1))
                if 0<=a<self.n and 0<=b<self.n and not self.leaves[f'{a},{b}'].blocked]


def validate_config(config):
    if config['schema_version'] != 1 or config['data_kind'] != 'synthetic_harbor_not_long_beach':
        raise ValueError('Only schema 1 synthetic harbor configs supported')
    if config['units'] != {'distance':'m','angle':'rad','x_axis':'right','y_axis':'up'}:
        raise ValueError('Unsupported coordinate convention')
    bounds,obstacles = config['map']['bounds'],config['map']['obstacles']
    QuadTree(bounds,obstacles,config['map']['max_depth'])
    for key in ('maximum_curvature','sample_step_m','rs_anchor_step_m'):
        if not math.isfinite(config[key]) or config[key]<=0:
            raise ValueError(f'{key} must be finite and positive')
    if config['reverse_weight']<1 or not math.isfinite(config['reverse_weight']):
        raise ValueError('Invalid reverse weight')
    if not (0<=config['validation_clearance_m']<=config['planning_clearance_m']):
        raise ValueError('Planning clearance must cover validation clearance')
    for scenario in config['scenarios']:
        for key in ('start','goal'):
            if len(scenario[key])!=3 or not all(math.isfinite(v) for v in scenario[key]):
                raise ValueError('Pose must be three finite values')
        if scenario['final_direction'] not in (None,-1,1):
            raise ValueError('final_direction must be null, -1, or 1')


def run(config):
    validate_config(config)
    bounds,obstacles = config['map']['bounds'],config['map']['obstacles']
    def inflated(margin):
        return [(max(bounds[0],l-margin),max(bounds[1],b-margin),
                 min(bounds[2],r+margin),min(bounds[3],t+margin)) for l,b,r,t in obstacles]
    plan_boxes = inflated(config['planning_clearance_m'])
    check_boxes = inflated(config['validation_clearance_m'])
    t0 = time.perf_counter()
    tree = QuadTree(bounds,plan_boxes,config['map']['max_depth'])
    grid = UniformGrid(bounds,plan_boxes,config['map']['max_depth'])
    map_seconds = time.perf_counter()-t0
    results = []
    for scenario in config['scenarios']:
        start,goal = scenario['start'],scenario['goal']
        baseline = astar(grid,start[:2],goal[:2])
        multi = astar(tree,start[:2],goal[:2])
        improved = astar(tree,start[:2],goal[:2],any_angle=True)
        reduced = shortcut(tree,improved['path'])
        smooth = g2_smooth(reduced,config['maximum_curvature'])
        if not collision_free(smooth,check_boxes,bounds):
            raise ValueError(f'{scenario["name"]}: smoothing collides; revise map/clearance, not the evidence label')
        # Anchors are actual positions on analytic pieces, with exact split support.
        # Arc length is approximated for choosing anchors; not a simulation of sailing.
        anchors,elapsed = [],0.0
        for index,piece in enumerate(smooth):
            samples = piece.samples(config['sample_step_m'])
            for k,(a,b) in enumerate(zip(samples,samples[1:])):
                elapsed += math.dist(a[:2],b[:2])
                anchors.append((elapsed,index,(k+1)/(len(samples)-1),b[:3]))
        total = elapsed
        starts = [a for i,a in enumerate(anchors) if a[0]<=total/3 and (i==0 or int(a[0]/config['rs_anchor_step_m'])!=int(anchors[i-1][0]/config['rs_anchor_step_m']))]
        ends = [a for i,a in enumerate(anchors) if a[0]>=2*total/3 and (i==0 or int(a[0]/config['rs_anchor_step_m'])!=int(anchors[i-1][0]/config['rs_anchor_step_m']))]
        t1 = time.perf_counter()
        begin_options = []
        for anchor in starts:
            connection = connect(start,anchor[3],config['maximum_curvature'],check_boxes,bounds,config['reverse_weight'],1,1)
            if connection:
                begin_options.append((connection['cost']-anchor[0],anchor,connection))
        end_options = []
        for anchor in ends:
            connection = connect(anchor[3],goal,config['maximum_curvature'],check_boxes,bounds,config['reverse_weight'],scenario['final_direction'],1)
            # Forward entry maintains tangent direction at the middle-to-RS join.
            if connection and connection['pieces'][0].direction==1:
                end_options.append((connection['cost']+anchor[0],anchor,connection))
        if not begin_options or not end_options:
            raise ValueError(f'{scenario["name"]}: no feasible start/end RS connection')
        _,first,initial = min(begin_options,key=lambda x:x[0])
        # In the last third try anchors in sailing order, as described in p.44.
        _,last,terminal = end_options[0]
        middle = trim(smooth,first[1],first[2],last[1],last[2])
        full = initial['pieces']+middle+terminal['pieces']
        samples = sample_path(full,config['sample_step_m'])
        positions = [p[:2] for p in samples]
        length = sum(math.dist(a,b) for a,b in zip(positions,positions[1:]))
        reverse = sum(p.length for p in full if p.direction<0)
        max_k = max(abs(p[3]) for p in samples)
        max_extrema = max(curvature_maximum(p) for p in full)
        joins = []
        cusps = []
        for a,b in zip(full,full[1:]):
            u,v = a.evaluate(1),b.evaluate(0)
            switch = a.direction != b.direction
            joins.append(dict(position_gap_m=math.dist(u[:2],v[:2]),yaw_gap_rad=abs(wrap(u[2]-v[2])),curvature_jump=abs(u[3]-v[3]),direction_switch=switch))
            if switch:
                cusps.append(u[:2])
        passed = collision_free(full,check_boxes,bounds)
        passed = passed and math.dist(samples[0][:2],start[:2])<1e-5 and math.dist(samples[-1][:2],goal[:2])<1e-5
        passed = passed and abs(wrap(samples[0][2]-start[2]))<1e-7 and abs(wrap(samples[-1][2]-goal[2]))<1e-7
        passed = passed and max_extrema<=config['maximum_curvature']*(1+1e-4)
        passed = passed and all(j['position_gap_m']<1e-5 and j['yaw_gap_rad']<1e-7 for j in joins)
        if scenario['final_direction'] is not None:
            passed = passed and full[-1].direction==scenario['final_direction']
        if not passed:
            raise ValueError(f'{scenario["name"]}: full path failed independent validation')
        results.append(dict(name=scenario['name'],baseline=baseline,multi=multi,improved=improved,
                            reduced=reduced,smooth=sample_path(smooth,config['sample_step_m']),
                            smooth_pieces=smooth,pieces=full,samples=samples,cusps=cusps,
                            metrics=dict(validation_passed=passed,length_m=length,reverse_m=reverse,
                            maximum_curvature_sampled=max_k,maximum_curvature_extrema=max_extrema,cusps=len(cusps),joins=joins,
                            final_direction=full[-1].direction,rs_seconds=time.perf_counter()-t1,
                            start_anchor_m=first[0],end_anchor_m=last[0],spine_length_m=total,
                            start_rs=initial['modes'],end_rs=terminal['modes'],
                            start_signed_lengths_m=initial['signed_lengths_m'],end_signed_lengths_m=terminal['signed_lengths_m'])))
    return tree,grid,map_seconds,results


def trim(pieces,start_index,start_t,end_index,end_t):
    """Exact Bezier de Casteljau splitting at RS joins, not polyline substitution."""
    result = []
    for i in range(start_index,end_index+1):
        p = pieces[i]
        lo,hi = (start_t if i==start_index else 0),(end_t if i==end_index else 1)
        if hi-lo<1e-10:
            continue
        if p.kind != 'cubic':
            result.append(replace(p,pose=p.evaluate(lo)[:3],length=p.length*(hi-lo)))
        else:
            controls = p.controls
            for t,side in ((hi,'left'),(lo/hi,'right')):
                a,b,c,d = controls
                ab,bc,cd = [tuple((1-t)*v[j]+t*w[j] for j in range(2)) for v,w in ((a,b),(b,c),(c,d))]
                abc,bcd = [tuple((1-t)*v[j]+t*w[j] for j in range(2)) for v,w in ((ab,bc),(bc,cd))]
                middle = tuple((1-t)*abc[j]+t*bcd[j] for j in range(2))
                controls = (a,ab,abc,middle) if side=='left' else (middle,bcd,cd,d)
            result.append(replace(p,controls=controls))
    return result
