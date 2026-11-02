"""Bounded forward pose-lattice search with analytic Dubins goal connections.

Actual continuous poses and immutable parents are retained; state bins only
prune search alternatives. Returned paths are G1 and checked primitive by
primitive. Finite discretization/budget failure is not an infeasibility proof.
"""
import heapq
import math
from .curves import Piece,wrap
from .dubins import candidates


def forward_pose_search(start,goal,radius,is_clear,*,bounds,position_resolution=2.5,heading_bins=24,
                        straight_step=5.,max_expansions=50000,shot_every=25,shot_distance=250.):
    if (len(start)!=3 or len(goal)!=3 or not all(math.isfinite(v)for v in (*start,*goal,radius,position_resolution,straight_step,*bounds))
            or min(radius,position_resolution,straight_step)<=0 or heading_bins<4 or max_expansions<1 or shot_every<1):
        raise ValueError('Finite poses and positive search settings required')
    def key(p):return(round(p[0]/position_resolution),round(p[1]/position_resolution),round((p[2]%(2*math.pi))*heading_bins/(2*math.pi))%heading_bins)
    def h(p):return max(math.dist(p[:2],goal[:2]),radius*abs(wrap(p[2]-goal[2])))
    # Each node owns its actual pose and immutable parent index. Replacing a
    # quantized state's best candidate must never rewrite existing ancestry.
    nodes=[(tuple(start),0.,None,None)];best={key(start):0};queue=[(h(start),0)];expanded=0
    while queue and expanded<max_expansions:
        _,index=heapq.heappop(queue);pose,cost,parent,piece=nodes[index]
        if best.get(key(pose))!=index:continue
        expanded+=1
        if (expanded-1)%shot_every==0 and math.dist(pose[:2],goal[:2])<=shot_distance:
            for path in candidates(pose,goal,radius):
                if not is_clear(path['pieces']):continue
                prefix=[];cursor=index
                while nodes[cursor][2]is not None:
                    prefix.append(nodes[cursor][3]);cursor=nodes[cursor][2]
                pieces=[]
                for p in prefix[::-1]+path['pieces']:
                    if pieces and pieces[-1].curvature==p.curvature:
                        prior=pieces[-1]
                        pieces[-1]=Piece(prior.kind,pose=prior.pose,length=prior.length+p.length,curvature=prior.curvature)
                    else:pieces.append(p)
                return {'passed':True,'pieces':pieces,'length_m':sum(p.length for p in pieces),'expanded_states':expanded,'generated_nodes':len(nodes),'word':'bounded_forward_pose_search'}
        for turn in (0,1,-1):
            step=straight_step if turn==0 else radius*2*math.pi/heading_bins
            p=Piece('line'if turn==0 else 'arc',pose=pose,length=step,curvature=turn/radius)
            end=p.evaluate(1)[:3]
            if not(bounds[0]<=end[0]<=bounds[2]and bounds[1]<=end[1]<=bounds[3]):continue
            signature=key(end);newcost=cost+step
            if signature in best and nodes[best[signature]][1]<=newcost+1e-9:continue
            if not is_clear([p]):continue
            nodes.append((end,newcost,index,p));newindex=len(nodes)-1;best[signature]=newindex
            heapq.heappush(queue,(newcost+h(end),newindex))
    return {'passed':False,'expanded_states':expanded,'generated_nodes':len(nodes),'search_exhausted':not bool(queue),
            'scope':'Finite forward lattice, bounded domain and expansion cap; not proof of continuous-space infeasibility'}
