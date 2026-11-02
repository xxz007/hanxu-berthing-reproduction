"""Forward-only six-family Dubins connectors, adapted from PythonRobotics.

Scalar formulas: Atsushi Sakai and contributors, MIT, pinned commit
8c392897c7f41a6e4d9aab9cb2e9737a7a6fa031. See third_party/PythonRobotics.LICENSE
and docs/dubins_sources_a5.json. Own pose integration and endpoint checks.
No obstacle avoidance or full-vehicle feasibility is implied by enumeration.
"""
import math
from math import sin, cos, atan2, acos, sqrt, pi
from .curves import Piece, wrap


def _mod2pi(theta):
    value=theta % (2*pi)
    return 0.0 if min(value,2*pi-value)<1e-12 else value


def _calc_trig_funcs(alpha, beta):
    sin_a = sin(alpha)
    sin_b = sin(beta)
    cos_a = cos(alpha)
    cos_b = cos(beta)
    cos_ab = cos(alpha - beta)
    return sin_a, sin_b, cos_a, cos_b, cos_ab


def _LSL(alpha, beta, d):
    sin_a, sin_b, cos_a, cos_b, cos_ab = _calc_trig_funcs(alpha, beta)
    mode = ["L", "S", "L"]
    p_squared = 2 + d ** 2 - (2 * cos_ab) + (2 * d * (sin_a - sin_b))
    if p_squared < 0:  # invalid configuration
        return None, None, None, mode
    tmp = atan2((cos_b - cos_a), d + sin_a - sin_b)
    d1 = _mod2pi(-alpha + tmp)
    d2 = sqrt(p_squared)
    d3 = _mod2pi(beta - tmp)
    return d1, d2, d3, mode


def _RSR(alpha, beta, d):
    sin_a, sin_b, cos_a, cos_b, cos_ab = _calc_trig_funcs(alpha, beta)
    mode = ["R", "S", "R"]
    p_squared = 2 + d ** 2 - (2 * cos_ab) + (2 * d * (sin_b - sin_a))
    if p_squared < 0:
        return None, None, None, mode
    tmp = atan2((cos_a - cos_b), d - sin_a + sin_b)
    d1 = _mod2pi(alpha - tmp)
    d2 = sqrt(p_squared)
    d3 = _mod2pi(-beta + tmp)
    return d1, d2, d3, mode


def _LSR(alpha, beta, d):
    sin_a, sin_b, cos_a, cos_b, cos_ab = _calc_trig_funcs(alpha, beta)
    p_squared = -2 + d ** 2 + (2 * cos_ab) + (2 * d * (sin_a + sin_b))
    mode = ["L", "S", "R"]
    if p_squared < 0:
        return None, None, None, mode
    d1 = sqrt(p_squared)
    tmp = atan2((-cos_a - cos_b), (d + sin_a + sin_b)) - atan2(-2.0, d1)
    d2 = _mod2pi(-alpha + tmp)
    d3 = _mod2pi(-_mod2pi(beta) + tmp)
    return d2, d1, d3, mode


def _RSL(alpha, beta, d):
    sin_a, sin_b, cos_a, cos_b, cos_ab = _calc_trig_funcs(alpha, beta)
    p_squared = d ** 2 - 2 + (2 * cos_ab) - (2 * d * (sin_a + sin_b))
    mode = ["R", "S", "L"]
    if p_squared < 0:
        return None, None, None, mode
    d1 = sqrt(p_squared)
    tmp = atan2((cos_a + cos_b), (d - sin_a - sin_b)) - atan2(2.0, d1)
    d2 = _mod2pi(alpha - tmp)
    d3 = _mod2pi(beta - tmp)
    return d2, d1, d3, mode


def _RLR(alpha, beta, d):
    sin_a, sin_b, cos_a, cos_b, cos_ab = _calc_trig_funcs(alpha, beta)
    mode = ["R", "L", "R"]
    tmp = (6.0 - d ** 2 + 2.0 * cos_ab + 2.0 * d * (sin_a - sin_b)) / 8.0
    if abs(tmp) > 1.0:
        return None, None, None, mode
    d2 = _mod2pi(2 * pi - acos(tmp))
    d1 = _mod2pi(alpha - atan2(cos_a - cos_b, d - sin_a + sin_b) + d2 / 2.0)
    d3 = _mod2pi(alpha - beta - d1 + d2)
    return d1, d2, d3, mode


def _LRL(alpha, beta, d):
    sin_a, sin_b, cos_a, cos_b, cos_ab = _calc_trig_funcs(alpha, beta)
    mode = ["L", "R", "L"]
    tmp = (6.0 - d ** 2 + 2.0 * cos_ab + 2.0 * d * (- sin_a + sin_b)) / 8.0
    if abs(tmp) > 1.0:
        return None, None, None, mode
    d2 = _mod2pi(2 * pi - acos(tmp))
    d1 = _mod2pi(-alpha - atan2(cos_a - cos_b, d + sin_a - sin_b) + d2 / 2.0)
    d3 = _mod2pi(_mod2pi(beta) - alpha - d1 + _mod2pi(d2))
    return d1, d2, d3, mode



def candidates(start, goal, minimum_radius_m):
    """All valid forward CSC/CCC words sorted by analytic length."""
    if (len(start)!=3 or len(goal)!=3 or not all(math.isfinite(v)for v in (*start,*goal,minimum_radius_m))
            or minimum_radius_m<=0):
        raise ValueError('Finite 2D poses and positive minimum radius required')
    if math.dist(start[:2],goal[:2])<1e-9 and abs(wrap(goal[2]-start[2]))<1e-9:
        return [{'word':'identity','length_m':0.0,'pieces':[]}]
    dx,dy=goal[0]-start[0],goal[1]-start[1]
    theta=math.atan2(dy,dx);distance=math.hypot(dx,dy)/minimum_radius_m
    alpha,beta=_mod2pi(start[2]-theta),_mod2pi(goal[2]-theta)
    paths=[]
    # Exact single-circle degeneracy: a zero straight tangent between equal
    # circles makes atan2 ill-conditioned in a generic CSC formula.
    for sign in (1,-1):
        angle=_mod2pi(sign*(goal[2]-start[2]))
        if angle==0:continue
        piece=Piece('arc',pose=tuple(start),length=angle*minimum_radius_m,curvature=sign/minimum_radius_m)
        if math.dist(piece.evaluate(1)[:2],goal[:2])<1e-7:
            paths.append({'word':'L'if sign==1 else 'R','length_m':piece.length,'pieces':[piece]})
    for function in (_LSL,_RSR,_LSR,_RSL,_RLR,_LRL):
        t,p,q,modes=function(alpha,beta,distance)
        if t is None:continue
        pose=tuple(start);pieces=[]
        for value,mode in zip((t,p,q),modes):
            length=value*minimum_radius_m
            if length<1e-9:continue
            curvature=(1 if mode=='L'else -1 if mode=='R'else 0)/minimum_radius_m
            piece=Piece('arc'if curvature else 'line',pose=pose,length=length,curvature=curvature)
            pieces.append(piece);pose=piece.evaluate(1)[:3]
        if math.dist(pose[:2],goal[:2])>1e-5 or abs(wrap(pose[2]-goal[2]))>1e-7:continue
        paths.append({'word':''.join(modes),'length_m':sum(p.length for p in pieces),'pieces':pieces})
    return sorted(paths,key=lambda p:(p['length_m'],p['word']))
