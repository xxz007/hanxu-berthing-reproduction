"""2008 Yang/Sukkarieh Fig.8 angle bisection, using 2010 cubic pairs.

This is an explicit reference-backed extension, not a recovered implementation
of Han's unspecified merging of adjacent short turns.
"""
import math
from .curves import g2_smooth


def bisect_corners(path, maximum_curvature):
    """Split each turn into two half-angle G2CBS corners.

    In the 2008 paper beta=gamma/2 and alpha=gamma/4. Place two virtual
    vertices L_b=d_b/cos(beta) away from the original corner, one on each
    incident ray. Their connecting chord is exactly 2*d_b long, so its two
    symmetric corner trims meet at zero curvature. Each original incident
    segment needs d_b+L_b. Check that budget *before* changing the vertices.

    d_b uses the full 2010 c2/c3 expression rather than the rounded 2008
    coefficient 1.1228. Output retains original/virtual vertices and budgets.
    It does not check obstacles; the caller must check every resulting curve.
    """
    if not math.isfinite(maximum_curvature) or maximum_curvature<=0:
        raise ValueError('Maximum curvature must be finite and positive')
    if len(path)<2 or any(len(p)!=2 or not all(math.isfinite(x) for x in p) for p in path):
        raise ValueError('Path must contain at least two finite 2D points')
    path=[tuple(p) for p in path]
    lengths=[math.dist(a,b) for a,b in zip(path,path[1:])]
    if any(length<=1e-8 for length in lengths):
        raise ValueError('Consecutive path points must be distinct')
    c2=2*(math.sqrt(6)-1)/5
    c3=(c2+4)/(7.2364+6)
    coefficient=(c2+4)**2/(54*c3)
    trims=[0.0]*len(path)
    corners=[]
    for i in range(1,len(path)-1):
        a,b,c=path[i-1:i+2]
        u=tuple((b[j]-a[j])/lengths[i-1] for j in (0,1))
        v=tuple((c[j]-b[j])/lengths[i] for j in (0,1))
        gamma=math.atan2(u[0]*v[1]-u[1]*v[0],sum(u[j]*v[j] for j in (0,1)))
        if abs(gamma)<1e-7:
            corners.append({'original_index':i,'gamma_rad':gamma,'d_b_m':0.0,
                            'L_b_m':0.0,'required_trim_m':0.0,'virtual_vertices':[b]})
            continue
        if abs(gamma)>math.pi-1e-6:
            raise ValueError('G2CBS bisection cannot smooth a reversal')
        beta=abs(gamma)/2
        alpha=beta/2
        d_b=coefficient*math.sin(alpha)/(maximum_curvature*math.cos(alpha)**2)
        L_b=d_b/math.cos(beta)
        trims[i]=d_b+L_b
        corners.append({'original_index':i,'gamma_rad':gamma,'d_b_m':d_b,
                        'L_b_m':L_b,'required_trim_m':trims[i],
                        'virtual_vertices':[tuple(b[j]-L_b*u[j] for j in (0,1)),
                                            tuple(b[j]+L_b*v[j] for j in (0,1))]})
    budgets=[{'original_segment_index':i,'length_m':length,
              'required_trim_m':trims[i]+trims[i+1],
              'remaining_m':length-trims[i]-trims[i+1]} for i,length in enumerate(lengths)]
    failed=[b['original_segment_index'] for b in budgets if b['remaining_m'] < -1e-8]
    virtual_path=[path[0]]
    for corner in corners:
        virtual_path.extend(corner['virtual_vertices'])
    virtual_path.append(path[-1])
    result={'original_path':path,'virtual_path':virtual_path,'corners':corners,
            'segment_budgets':budgets,'insufficient_segments':failed,
            'status':'insufficient_segment_budget' if failed else 'constructed',
            'maximum_curvature_per_m':maximum_curvature,'pieces':[]}
    if not failed:
        result['pieces']=g2_smooth(virtual_path,maximum_curvature)
    return result
