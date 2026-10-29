"""Explicit feasible RS primitive sequence for stern approach to a berth axis.

This S+ C+ | S- family is an independent obstacle-aware extension, not one of
the canonical obstacle-free shortest RS words. No global optimality claim.
"""
import math
from .curves import Piece,collision_free,wrap


def connect_stern_axis(start,goal,maximum_curvature,obstacles,bounds,radii,reverse_weight=2.0):
    """Forward straight + one forward turn + reverse along goal heading.

    Solve lateral endpoint displacement for the initial straight length in
    the goal frame, using the thesis eq.(3.3) primitives. Candidate radii are
    caller-supplied engineering settings, each >= the minimum allowed radius.
    """
    if maximum_curvature<=0 or reverse_weight<1:
        raise ValueError('Positive curvature and reverse weight >=1 required')
    if any(not math.isfinite(r) or r<1/maximum_curvature for r in radii):
        raise ValueError('Candidate radii must respect the minimum turn radius')
    c,s=math.cos(goal[2]),math.sin(goal[2])
    dx,dy=start[0]-goal[0],start[1]-goal[1]
    x,y=c*dx+s*dy,-s*dx+c*dy
    theta=wrap(start[2]-goal[2])
    delta=wrap(-theta)
    # Parallel/antiparallel degeneracies are left to canonical RS candidates.
    if abs(math.sin(theta))<1e-8:
        return None
    options=[]
    for radius in radii:
        k=math.copysign(1/radius,delta)
        forward=-(y+(math.cos(theta)-1)/k)/math.sin(theta)
        reverse=x+forward*math.cos(theta)-math.sin(theta)/k
        if forward<0 or reverse<=1e-8:
            continue
        pieces=[]
        pose=tuple(start)
        if forward>1e-8:
            p=Piece('line',pose=pose,length=forward)
            pieces.append(p);pose=p.evaluate(1)[:3]
        p=Piece('arc',pose=pose,length=radius*abs(delta),curvature=k)
        pieces.append(p);pose=p.evaluate(1)[:3]
        pieces.append(Piece('line',pose=pose,length=reverse,direction=-1))
        end=pieces[-1].evaluate(1)
        if math.dist(end[:2],goal[:2])>1e-5 or abs(wrap(end[2]-goal[2]))>1e-7:
            raise AssertionError('Stern axis primitive derivation failed endpoint check')
        if not collision_free(pieces,obstacles,bounds):
            continue
        length=forward+radius*abs(delta)+reverse
        options.append({'pieces':pieces,'length_m':length,'reverse_m':reverse,
                        'cost':length+(reverse_weight-1)*reverse,
                        'modes':('S' if forward>1e-8 else '')+('L' if k>0 else 'R')+'S',
                        'signed_lengths_m':([forward] if forward>1e-8 else [])+[radius*abs(delta),-reverse],
                        'candidate_kind':'independent_forward_arc_stern_axis_extension',
                        'turn_radius_m':radius})
    return min(options,key=lambda p:p['cost']) if options else None
