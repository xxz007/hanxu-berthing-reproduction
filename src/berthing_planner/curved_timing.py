"""Continuous-distance certificates for forward curves with variable speed.

Intervals are enclosed using travelled arc length, not straight interpolation
or a sample-only collision check. Numerical margins expand every bound.
"""
from bisect import bisect_left
import heapq
import math
from .curves import Piece,wrap


class CurvedClock:
    def __init__(self,pieces,profile,departure_s=0.,initial_pose=None):
        if not math.isfinite(departure_s)or departure_s<0:raise ValueError('Nonnegative finite departure required')
        self.pieces=tuple(Piece(**p)if isinstance(p,dict)else p for p in pieces)
        self.profile=profile;self.departure_s=departure_s;self.duration_s=profile['duration_s'];self.finish_s=departure_s+self.duration_s
        self.segments=profile['segments'];self.ends=tuple(s['t1_s']for s in self.segments)
        self.total_join_position_error_m=0.
        if len(self.pieces)!=len(self.segments):raise ValueError('Curve/profile count mismatch')
        if not self.pieces:
            if initial_pose is None or len(initial_pose)!=3 or not all(math.isfinite(v)for v in initial_pose):raise ValueError('Finite stationary pose required')
            if self.duration_s!=0 or profile['length_m']!=0:raise ValueError('Stationary clock has no motion')
            self.initial_pose=tuple(initial_pose);self.final_pose=self.initial_pose;self.phase_times=(departure_s,);return
        end=length=0.;phases={departure_s}
        for i,(p,s)in enumerate(zip(self.pieces,self.segments)):
            if p.kind not in ('line','arc')or p.direction!=1 or p.length<=0 or len(p.pose)!=3 or not all(math.isfinite(v)for v in (*p.pose,p.length,p.curvature)):
                raise ValueError('Positive finite forward primitives required')
            values=[s[k]for k in ('t0_s','t1_s','s0_m','length_m','peak_speed_mps','ramp_duration_s','cruise_duration_s','accel_mps2','ramp_distance_m')]
            if not all(math.isfinite(v)for v in values):raise ValueError('Finite profile required')
            ta,tc,v,a=s['ramp_duration_s'],s['cruise_duration_s'],s['peak_speed_mps'],s['accel_mps2']
            if min(ta,v,a)<=0 or tc<0 or abs(s['t0_s']-end)>1e-7 or abs(s['s0_m']-length)>1e-6 or s['piece_index']!=i:
                raise ValueError('Contiguous monotone profile required')
            if max(abs(2*ta+tc-(s['t1_s']-s['t0_s'])),abs(v-a*ta))>1e-7 or max(abs(v*(ta+tc)-p.length),abs(s['length_m']-p.length),abs(s['ramp_distance_m']-.5*a*ta*ta))>1e-6:
                raise ValueError('Profile integrals do not match curve')
            if i and(math.dist(self.pieces[i-1].evaluate(1)[:2],p.pose[:2])>1e-6 or abs(wrap(self.pieces[i-1].evaluate(1)[2]-p.pose[2]))>1e-8):
                raise ValueError('Continuous position/heading required')
            if i:self.total_join_position_error_m+=math.dist(self.pieces[i-1].evaluate(1)[:2],p.pose[:2])
            phases.update(departure_s+t for t in (s['t0_s'],s['t0_s']+ta,s['t0_s']+ta+tc,s['t1_s']))
            end=s['t1_s'];length+=p.length
        if abs(end-self.duration_s)>1e-7 or abs(length-profile['length_m'])>1e-6:raise ValueError('Profile totals differ')
        self.initial_pose=self.pieces[0].evaluate(0)[:3];self.final_pose=self.pieces[-1].evaluate(1)[:3];self.phase_times=tuple(sorted(phases))

    @classmethod
    def stationary(cls,pose):return cls([],{'segments':[],'duration_s':0.,'length_m':0.},initial_pose=pose)

    def state(self,time_s):
        if not math.isfinite(time_s):raise ValueError('Finite time required')
        local=time_s-self.departure_s
        if not self.pieces:return {'position_m':self.initial_pose[:2],'heading_rad':self.initial_pose[2],'distance_m':0.,'speed_mps':0.,'piece_index':None,'mode':'hold'}
        if local<=0:idx=0;distance=speed=0.;progress=0.;mode='predeparture_hold'if local<0 else'moving'
        elif local>=self.duration_s:idx=len(self.pieces)-1;distance=self.pieces[-1].length;progress=self.profile['length_m'];speed=0.;mode='completed_hold'
        else:
            idx=bisect_left(self.ends,local);s=self.segments[idx];t=local-s['t0_s'];ta,tc=s['ramp_duration_s'],s['cruise_duration_s'];a,v=s['accel_mps2'],s['peak_speed_mps']
            if t<=ta:distance=.5*a*t*t;speed=a*t
            elif t<=ta+tc:distance=s['ramp_distance_m']+v*(t-ta);speed=v
            else:
                remaining=s['t1_s']-local;distance=s['length_m']-.5*a*remaining*remaining;speed=a*remaining
            progress=s['s0_m']+distance;mode='moving'
        pose=self.pieces[idx].evaluate(distance/self.pieces[idx].length)
        return {'position_m':pose[:2],'heading_rad':pose[2],'distance_m':progress,'speed_mps':speed,'piece_index':idx,'mode':mode}


def continuous_distance_bounds(first,second,end_s,*,tolerance_m=.05,numerical_margin_m=1e-6,max_refinements=200000):
    """Bound global min/max distance on[0,end] via arc-length branch and bound.

    For midpoint m, a side's sum of travelled lengths bounds the change of
    relative position at every time on that side, including holds and arcs.
    Endpoint/midpoint witnesses bracket extrema; returned intervals cover all
    times. A finite-budget nonconvergence is explicit, never a safety pass.
    """
    if not all(math.isfinite(v)and v>0 for v in (end_s,tolerance_m,numerical_margin_m))or tolerance_m<2*numerical_margin_m or max_refinements<0:
        raise ValueError('Positive finite horizon/tolerance and valid budget required')
    times=sorted({0.,end_s,*[t for clock in(first,second)for t in clock.phase_times if 0<t<end_s]})
    cache={};nodes={};minheap=[];maxheap=[];next_id=0;best_min=math.inf;best_max=-math.inf;min_time=max_time=0.
    def sample(t):
        nonlocal best_min,best_max,min_time,max_time
        if t not in cache:
            a,b=first.state(t),second.state(t);d=math.dist(a['position_m'],b['position_m']);cache[t]=(d,a['distance_m'],b['distance_m'])
            if d<best_min:best_min=d;min_time=t
            if d>best_max:best_max=d;max_time=t
        return cache[t]
    def add(lo,hi):
        nonlocal next_id
        mid=(lo+hi)/2;left=sample(lo);centre=sample(mid);right=sample(hi)
        movement=max(0.,centre[1]-left[1]+centre[2]-left[2],right[1]-centre[1]+right[2]-centre[2])+numerical_margin_m+first.total_join_position_error_m+second.total_join_position_error_m
        lower=max(0.,centre[0]-movement);upper=centre[0]+movement
        node={'t0_s':lo,'t1_s':hi,'midpoint_s':mid,'midpoint_distance_m':centre[0],'relative_movement_bound_m':movement,'minimum_distance_lower_m':lower,'maximum_distance_upper_m':upper}
        next_id+=1;nodes[next_id]=node;heapq.heappush(minheap,(lower,next_id));heapq.heappush(maxheap,(-upper,next_id))
    for lo,hi in zip(times,times[1:]):
        if hi>lo:add(lo,hi)
    refinements=0;converged=False
    while nodes:
        while minheap[0][1]not in nodes:heapq.heappop(minheap)
        while maxheap[0][1]not in nodes:heapq.heappop(maxheap)
        low=minheap[0][0];high=-maxheap[0][0];min_gap=best_min-low;max_gap=high-best_max
        if max(min_gap,max_gap)<=tolerance_m:converged=True;break
        if refinements>=max_refinements:break
        index=minheap[0][1]if min_gap>=max_gap else maxheap[0][1];node=nodes.pop(index);lo,hi=node['t0_s'],node['t1_s'];mid=node['midpoint_s']
        if mid==lo or mid==hi:
            nodes[index]=node;break
        add(lo,mid);add(mid,hi);refinements+=1
    partition=sorted(nodes.values(),key=lambda n:n['t0_s']);lower=min(n['minimum_distance_lower_m']for n in partition);upper=max(n['maximum_distance_upper_m']for n in partition)
    return {'converged':converged,'minimum_distance_lower_m':lower,'minimum_distance_witness_m':best_min,'minimum_witness_time_s':min_time,
            'maximum_distance_witness_m':best_max,'maximum_distance_upper_m':upper,'maximum_witness_time_s':max_time,
            'tolerance_m':tolerance_m,'numerical_margin_m':numerical_margin_m,'refinements':refinements,'state_sample_count':len(cache),'partition':partition,
            'method':'Whole-time arc-length movement enclosures; sample witnesses plus outward interval bounds, not sample-only acceptance'}
