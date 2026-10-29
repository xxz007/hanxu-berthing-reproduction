"""Analytic line, cubic Bezier and bounded-curvature arc primitives."""
from dataclasses import dataclass
import math


def wrap(angle):
    return (angle + math.pi) % (2*math.pi) - math.pi


@dataclass
class Piece:
    kind: str
    controls: tuple = ()
    pose: tuple = ()
    length: float = 0.0
    curvature: float = 0.0
    direction: int = 1

    def evaluate(self, t):
        if self.kind == 'cubic':
            p = self.controls
            xy = tuple(sum(p[i][j]*w for i,w in enumerate(((1-t)**3,3*t*(1-t)**2,3*t*t*(1-t),t**3))) for j in range(2))
            d = tuple(3*((p[1][j]-p[0][j])*(1-t)**2 + 2*(p[2][j]-p[1][j])*(1-t)*t + (p[3][j]-p[2][j])*t*t) for j in range(2))
            dd = tuple(6*((p[2][j]-2*p[1][j]+p[0][j])*(1-t)+(p[3][j]-2*p[2][j]+p[1][j])*t) for j in range(2))
            speed = math.hypot(*d)
            if speed < 1e-10:
                raise ValueError('Degenerate Bezier tangent')
            k = (d[0]*dd[1]-d[1]*dd[0])/speed**3
            return (*xy, math.atan2(d[1],d[0]), k, 1)
        x,y,yaw = self.pose
        distance = self.direction*self.length*t
        if self.curvature == 0:
            x += distance*math.cos(yaw)
            y += distance*math.sin(yaw)
        else:
            final = yaw+self.curvature*distance
            x += (math.sin(final)-math.sin(yaw))/self.curvature
            y += (-math.cos(final)+math.cos(yaw))/self.curvature
            yaw = final
        return x,y,wrap(yaw),self.curvature,self.direction

    def samples(self, step=5.0):
        size = self.length if self.kind != 'cubic' else sum(math.dist(a,b) for a,b in zip(self.controls,self.controls[1:]))
        count = max(1, math.ceil(size/step))
        return [self.evaluate(i/count) for i in range(count+1)]


def line(a,b):
    return Piece('line',pose=(*a,math.atan2(b[1]-a[1],b[0]-a[0])),length=math.dist(a,b))


def g2_smooth(path, maximum_curvature):
    """Basic symmetric G2CBS corner pair, adapted from MIT MATLAB reference.

    Reject overlapping/too-short corners. Does not implement the thesis's
    unspecified merging of multiple short turns. Never clamp d and hide k violation.
    """
    c1, c2 = 7.2364, 2*(math.sqrt(6)-1)/5
    c3 = (c2+4)/(c1+6)
    corners = []
    for a,b,c in zip(path,path[1:],path[2:]):
        left,right = math.dist(a,b),math.dist(b,c)
        u = tuple((b[j]-a[j])/left for j in range(2))
        v = tuple((c[j]-b[j])/right for j in range(2))
        angle = math.acos(max(-1,min(1,sum(u[j]*v[j] for j in range(2)))))
        if angle < 1e-7:
            corners.append((0, b,b,[]))
            continue
        if angle > math.pi-1e-6:
            raise ValueError('G2CBS cannot smooth a reversal')
        beta = angle/2
        d = (c2+4)**2/(54*c3)*math.sin(beta)/(maximum_curvature*math.cos(beta)**2)
        h,g = c3*d,c2*c3*d
        B0 = tuple(b[j]-d*u[j] for j in range(2))
        B1 = tuple(B0[j]+g*u[j] for j in range(2))
        B2 = tuple(B1[j]+h*u[j] for j in range(2))
        E0 = tuple(b[j]+d*v[j] for j in range(2))
        E1 = tuple(E0[j]-g*v[j] for j in range(2))
        E2 = tuple(E1[j]-h*v[j] for j in range(2))
        mid = tuple((B2[j]+E2[j])/2 for j in range(2))
        corners.append((d,B0,E0,[Piece('cubic',controls=(B0,B1,B2,mid)),Piece('cubic',controls=(mid,E2,E1,E0))]))
    previous, pieces = path[0], []
    for i,(d,entry,exit,curves) in enumerate(corners):
        prior_d = corners[i-1][0] if i else 0
        if prior_d+d > math.dist(path[i],path[i+1])+1e-8:
            raise ValueError('G2CBS corners overlap: merge algorithm or more space needed')
        if math.dist(previous,entry)>1e-8:
            pieces.append(line(previous,entry))
        pieces.extend(curves)
        previous = exit
    if corners and corners[-1][0] > math.dist(path[-2],path[-1])+1e-8:
        raise ValueError('G2CBS last corner exceeds remaining segment')
    if math.dist(previous,path[-1])>1e-8:
        pieces.append(line(previous,path[-1]))
    return pieces


def sample_path(pieces,step=5):
    points = []
    for i,piece in enumerate(pieces):
        samples = piece.samples(step)
        points.extend(samples if i == 0 else samples[1:])
    return points


def segment_hits(a,b,box,padding=0):
    x0,y0,x1,y1 = box
    enter,leave = 0.0,1.0
    for p,q,lower,upper in ((a[0],b[0],x0-padding,x1+padding),(a[1],b[1],y0-padding,y1+padding)):
        delta = q-p
        if abs(delta)<1e-14:
            if p<lower or p>upper:
                return False
        else:
            lo,hi = sorted(((lower-p)/delta,(upper-p)/delta))
            enter,leave = max(enter,lo),min(leave,hi)
            if enter>leave:
                return False
    return True


def collision_free(pieces, obstacles, bounds, step=5):
    """Cubic convex-hull subdivision; arc chords inflated by sagitta.

    Conservative centerline checks against configured rectangle obstacles.
    No ship hull or dynamic model. Subdivision uncertainty is rejected.
    """
    x0,y0,x1,y1 = bounds
    for piece in pieces:
        if piece.kind == 'cubic':
            stack = [(piece.controls,0)]
            while stack:
                controls,depth = stack.pop()
                lo = [min(p[j] for p in controls) for j in range(2)]
                hi = [max(p[j] for p in controls) for j in range(2)]
                intersects = any(lo[0]<=r and l<=hi[0] and lo[1]<=t and b<=hi[1] for l,b,r,t in obstacles)
                inside = x0<=lo[0] and hi[0]<x1 and y0<=lo[1] and hi[1]<y1
                if any(l<=lo[0]<=hi[0]<=r and b<=lo[1]<=hi[1]<=t for l,b,r,t in obstacles):
                    return False
                if not intersects and inside:
                    continue
                if depth == 20:
                    return False
                a,b,c,d = controls
                ab,bc,cd = [tuple((p[j]+q[j])/2 for j in range(2)) for p,q in ((a,b),(b,c),(c,d))]
                abc,bcd = [tuple((p[j]+q[j])/2 for j in range(2)) for p,q in ((ab,bc),(bc,cd))]
                middle = tuple((abc[j]+bcd[j])/2 for j in range(2))
                stack.extend((((a,ab,abc,middle),depth+1),((middle,bcd,cd,d),depth+1)))
        else:
            samples = piece.samples(step)
            radius = 1/abs(piece.curvature) if piece.curvature else math.inf
            angle = piece.length/max(1,len(samples)-1)/radius
            sagitta = radius*(1-math.cos(angle/2)) if piece.curvature else 0
            for a,b in zip(samples,samples[1:]):
                if any(segment_hits(a,b,box,sagitta) for box in obstacles):
                    return False
                if not all(x0+sagitta<=p[0]<x1-sagitta and y0+sagitta<=p[1]<y1-sagitta for p in (a,b)):
                    return False
    return True


def polynomial_value(coefficients,t):
    value = 0.0
    for c in reversed(coefficients):
        value = value*t+c
    return value


def polynomial_roots_unit(coefficients):
    """Isolate real roots on [0,1] using derivative roots as monotone intervals."""
    scale = max((abs(v) for v in coefficients),default=0)
    if scale==0:
        return []
    p = [v/scale for v in coefficients]
    while len(p)>1 and abs(p[-1])<1e-14:
        p.pop()
    if len(p)<=1:
        return []
    critical = polynomial_roots_unit([i*p[i] for i in range(1,len(p))])
    knots = [0]+critical+[1]
    roots = [t for t in knots if abs(polynomial_value(p,t))<1e-11]
    for lo,hi in zip(knots,knots[1:]):
        low,high = polynomial_value(p,lo),polynomial_value(p,hi)
        if low*high>=0:
            continue
        for _ in range(55):
            mid = (lo+hi)/2
            value = polynomial_value(p,mid)
            if low*value<=0:
                hi,high = mid,value
            else:
                lo,low = mid,value
        roots.append((lo+hi)/2)
    return sorted(set(round(t,13) for t in roots))


def curvature_maximum(piece):
    """Numerical extrema of analytic cubic curvature, not just point sampling.

    k=N/Q^(3/2), stationary equation 2N'Q-3NQ'=0 (degree <=5).
    Floating point numerical check, not interval-arithmetic certification.
    """
    if piece.kind!='cubic':
        return abs(piece.curvature)
    p = piece.controls
    d = [[3*(p[1][j]-p[0][j]),6*(p[2][j]-2*p[1][j]+p[0][j]),3*(p[3][j]-3*p[2][j]+3*p[1][j]-p[0][j])] for j in range(2)]
    q = [sum(d[j][a]*d[j][b] for j in range(2) for a in range(3) for b in range(3) if a+b==k) for k in range(5)]
    n = [d[0][0]*d[1][1]-d[1][0]*d[0][1],2*(d[0][0]*d[1][2]-d[1][0]*d[0][2]),d[0][1]*d[1][2]-d[1][1]*d[0][2]]
    stationary = [0.0]*6
    for i in range(1,3):
        for j in range(5):
            stationary[i-1+j] += 2*i*n[i]*q[j]
    for i in range(3):
        for j in range(1,5):
            stationary[i+j-1] -= 3*n[i]*j*q[j]
    for t in [0,1]+polynomial_roots_unit([j*q[j] for j in range(1,5)]):
        if polynomial_value(q,t)<=1e-14:
            raise ValueError('Bezier has a stationary tangent')
    return max(abs(piece.evaluate(t)[3]) for t in [0,1]+polynomial_roots_unit(stationary))
