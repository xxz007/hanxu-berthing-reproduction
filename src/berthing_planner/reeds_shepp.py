"""Enumerate pinned RS analytic families, retain feasible weighted candidates."""
import math
from . import rs_families as families
from .curves import Piece, wrap, collision_free


def candidates(start, goal, maximum_curvature, reverse_weight=2.0, final_direction=None,initial_direction=None):
    if maximum_curvature <= 0 or reverse_weight < 1:
        raise ValueError('Positive curvature and reverse weight >= 1 required')
    dx,dy = goal[0]-start[0],goal[1]-start[1]
    c,s = math.cos(start[2]),math.sin(start[2])
    x,y,phi = (c*dx+s*dy)*maximum_curvature,(-s*dx+c*dy)*maximum_curvature,wrap(goal[2]-start[2])
    if math.hypot(x,y)<1e-9 and abs(phi)<1e-9:
        return []
    functions = [getattr(families,n) for n in (
        'left_straight_left','left_straight_right','left_x_right_x_left',
        'left_x_right_left','left_right_x_left','left_right_x_left_right',
        'left_x_right_left_x_right','left_x_right90_straight_left',
        'left_x_right90_straight_right','left_straight_right90_x_left',
        'left_straight_left90_x_right','left_x_right90_straight_left90_x_right')]
    paths, seen = [],set()
    for function in functions:
        for flip,reflect in ((False,False),(True,False),(False,True),(True,True)):
            tx = -x if flip else x
            ty = -y if reflect else y
            tp = -phi if flip != reflect else phi
            # CCC formulae have u1 in denominators; degenerate centered cases
            # are handled by the other analytic families, not by NaN/exception.
            singular = math.hypot(tx-math.sin(tp),ty-1+math.cos(tp))<1e-12
            if singular and function.__name__ == 'left_right_x_left':
                continue
            valid,lengths,modes = function(tx,ty,tp)
            if not valid:
                continue
            lengths = [(-1 if flip else 1)*v/maximum_curvature for v in lengths]
            modes = [('R' if m=='L' else 'L' if m=='R' else 'S') if reflect else m for m in modes]
            active = [(length,mode) for length,mode in zip(lengths,modes) if abs(length)>1e-8]
            if not active or (final_direction is not None and (1 if active[-1][0]>0 else -1)!=final_direction):
                continue
            if initial_direction is not None and (1 if active[0][0]>0 else -1)!=initial_direction:
                continue
            key = tuple((round(length,7),mode) for length,mode in active)
            if key in seen:
                continue
            seen.add(key)
            pose, pieces = tuple(start),[]
            for length,mode in active:
                k = maximum_curvature*(1 if mode=='L' else -1 if mode=='R' else 0)
                p = Piece('arc' if k else 'line',pose=pose,length=abs(length),curvature=k,direction=1 if length>0 else -1)
                pieces.append(p)
                pose = p.evaluate(1)[:3]
            residual = math.dist(pose[:2],goal[:2])
            if residual>1e-5 or abs(wrap(pose[2]-goal[2]))>1e-7:
                continue
            total = sum(abs(v) for v,_ in active)
            reverse = sum(-v for v,_ in active if v<0)
            paths.append(dict(pieces=pieces,length_m=total,reverse_m=reverse,
                              cost=total+(reverse_weight-1)*reverse,
                              modes=''.join(m for _,m in active),signed_lengths_m=[v for v,_ in active]))
    return sorted(paths,key=lambda p:p['cost'])


def connect(start,goal,maximum_curvature,obstacles,bounds,reverse_weight=2.0,final_direction=None,initial_direction=None):
    for candidate in candidates(start,goal,maximum_curvature,reverse_weight,final_direction,initial_direction):
        if collision_free(candidate['pieces'],obstacles,bounds):
            return candidate
    return None
