"""RS integration against the actually retained analytic spine, not its tail."""
import math
from .curves import curvature_maximum,collision_free,wrap
from .pipeline import trim
from .reeds_shepp import connect
from .stern_axis import connect_stern_axis


def piece_length(piece,end=1.0):
    """Arc length by converged composite Simpson integration; metre tolerance.

    Lines and arcs are exact. Cubic quadrature stops at a successive estimate
    difference below 1e-7 m, not a formal interval-arithmetic error bound.
    """
    if piece.kind!='cubic':
        return piece.length*end
    p=piece.controls
    def speed(t):
        return math.hypot(*(3*((p[1][j]-p[0][j])*(1-t)**2+
                  2*(p[2][j]-p[1][j])*(1-t)*t+(p[3][j]-p[2][j])*t*t) for j in (0,1)))
    previous=None
    for n in (8,16,32,64,128,256,512,1024,2048,4096):
        h=end/n
        value=h/3*(speed(0)+speed(end)+sum((4 if i%2 else 2)*speed(i*h) for i in range(1,n)))
        if previous is not None and abs(value-previous)<1e-7:
            return value
        previous=value
    raise ValueError('Cubic arc length did not converge')


def spine_anchor(pieces,lengths,distance):
    """Locate an arc-length anchor, retaining the exact cubic parameter."""
    if not 0<=distance<=sum(lengths)+1e-7:
        raise ValueError('Anchor outside available spine')
    elapsed=0.0
    for i,(piece,length) in enumerate(zip(pieces,lengths)):
        if distance<=elapsed+length+1e-8 or i==len(pieces)-1:
            target=max(0,min(length,distance-elapsed))
            if piece.kind!='cubic':
                t=target/length if length else 0
            else:
                lo,hi=0.0,1.0
                for _ in range(38):
                    mid=(lo+hi)/2
                    if piece_length(piece,mid)<target:
                        lo=mid
                    else:
                        hi=mid
                t=(lo+hi)/2
            return {'distance_m':distance,'index':i,'t':t,'pose':piece.evaluate(t)[:3]}
        elapsed+=length
    raise ValueError('Empty spine')


def stitch_rs(spine,start,goal,maximum_curvature,obstacles,bounds,reference_length,
              reverse_weight=2.0,final_direction=None,anchor_step=100.0,stern_axis_radii=()):
    """Test terminal RS at 100 m intervals in the last third of reference path.

    reference_length is explicitly the full A* polyline length; spine may be a
    shorter valid prefix. Initial RS sampling in the first third and its cost
    selection are engineering choices. No requirement that discarded tail
    geometry be collision-free; the complete retained result must pass.
    """
    if not spine or reference_length<=0 or anchor_step<=0:
        raise ValueError('Nonempty spine and positive reference length/step required')
    if stern_axis_radii and final_direction!=-1:
        raise ValueError('Stern-axis extension requires a reverse final direction')
    lengths=[piece_length(p) for p in spine]
    total=sum(lengths)
    starts=[]
    for j in range(1,math.floor(min(total,reference_length/3)/anchor_step)+1):
        a=spine_anchor(spine,lengths,j*anchor_step)
        connection=connect(start,a['pose'],maximum_curvature,obstacles,bounds,reverse_weight,1,1)
        if connection:
            starts.append((connection['cost']-a['distance_m'],a,connection))
    starts.sort(key=lambda item:item[0])
    trace=[]
    distance=2*reference_length/3
    while distance<=total+1e-7:
        end=spine_anchor(spine,lengths,min(distance,total))
        terminal=connect(end['pose'],goal,maximum_curvature,obstacles,bounds,reverse_weight,final_direction,1)
        if stern_axis_radii:
            alternate=connect_stern_axis(end['pose'],goal,maximum_curvature,obstacles,bounds,stern_axis_radii,reverse_weight)
            if alternate and (terminal is None or alternate['cost']<terminal['cost']):
                terminal=alternate
        step={'distance_m':distance,'pose':end['pose'],'terminal_rs_found':terminal is not None,
              'initial_options':len(starts),'retained_route_checks':0,
              'terminal_candidate_kind':terminal.get('candidate_kind','canonical_shortest_words') if terminal else None}
        trace.append(step)
        if terminal:
            for _,begin,initial in starts:
                if begin['distance_m']>=end['distance_m']:
                    continue
                middle=trim(spine,begin['index'],begin['t'],end['index'],end['t'])
                full=initial['pieces']+middle+terminal['pieces']
                step['retained_route_checks']+=1
                if not collision_free(full,obstacles,bounds):
                    continue
                joins=[(a.evaluate(1),b.evaluate(0),a.direction!=b.direction) for a,b in zip(full,full[1:])]
                max_k=max(curvature_maximum(p) for p in full)
                p,q=full[0].evaluate(0),full[-1].evaluate(1)
                if (max_k>maximum_curvature+1e-9 or
                    math.dist(p[:2],start[:2])>1e-5 or math.dist(q[:2],goal[:2])>1e-5 or
                    abs(wrap(p[2]-start[2]))>1e-7 or abs(wrap(q[2]-goal[2]))>1e-7 or
                    any(math.dist(a[:2],b[:2])>1e-5 or abs(wrap(a[2]-b[2]))>1e-7 for a,b,_ in joins)):
                    raise AssertionError('RS stitched geometry failed independent validation')
                return {'status':'retained_route_passed','pieces':full,'trace':trace,
                        'start_anchor':begin,'end_anchor':end,'spine_available_length_m':total,
                        'reference_polyline_length_m':reference_length,
                        'length_m':sum(piece_length(p) for p in full),
                        'reverse_m':sum(p.length for p in full if p.direction<0),
                        'maximum_curvature_per_m':max_k,
                        'cusps':[a[:2] for a,b,switch in joins if switch],
                        'maximum_curvature_jump_per_m':max((abs(a[3]-b[3]) for a,b,_ in joins),default=0),
                        'initial_rs':{k:v for k,v in initial.items() if k!='pieces'},
                        'terminal_rs':{k:v for k,v in terminal.items() if k!='pieces'}}
        distance+=anchor_step
    return {'status':'no_combination_in_this_finite_candidate_set','trace':trace,
            'pieces':[],'spine_available_length_m':total,'reference_polyline_length_m':reference_length}
