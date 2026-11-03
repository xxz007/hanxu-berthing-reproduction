"""Forward curved sorties separated by explicit ideal position holds."""
from bisect import bisect_right
import math
from .curved_timing import CurvedClock
from .curves import wrap


class ScheduledCurvedClock:
    def __init__(self,legs,initial_pose):
        if len(initial_pose)!=3 or not all(math.isfinite(v)for v in initial_pose):
            raise ValueError('Finite initial pose required')
        self.initial_pose=tuple(initial_pose);self.legs=[];self.offsets=[]
        self.total_join_position_error_m=0.;total=0.;end=0.;pose=self.initial_pose;phases={0.}
        for body in legs:
            c=CurvedClock(body['pieces'],body['profile'],body['departure_s'])
            if not c.pieces or c.departure_s<end-1e-8:raise ValueError('Nonempty nonoverlapping scheduled legs required')
            error=math.dist(pose[:2],c.initial_pose[:2])
            if error>1e-6 or abs(wrap(pose[2]-c.initial_pose[2]))>1e-8:raise ValueError('Scheduled leg position/heading discontinuity')
            self.total_join_position_error_m+=error+c.total_join_position_error_m
            self.legs.append(c);self.offsets.append(total);total+=c.profile['length_m'];end=c.finish_s;pose=c.final_pose;phases.update(c.phase_times)
        self.departures=tuple(c.departure_s for c in self.legs)
        self.final_pose=pose;self.finish_s=end;self.length_m=total;self.phase_times=tuple(sorted(phases))

    def state(self,time_s):
        if not math.isfinite(time_s):raise ValueError('Finite time required')
        i=bisect_right(self.departures,time_s)-1
        if i<0:return {'position_m':self.initial_pose[:2],'heading_rad':self.initial_pose[2],'distance_m':0.,'speed_mps':0.,'piece_index':None,'leg_index':None,'mode':'predeparture_hold'}
        s=self.legs[i].state(time_s);s['distance_m']+=self.offsets[i];s['leg_index']=i
        if time_s>=self.legs[i].finish_s:s['mode']='completed_hold'if i==len(self.legs)-1 else'interleg_hold'
        return s


class WindowClock:
    """Translate a known clock to a local time window for the same bound solver."""
    def __init__(self,clock,start_s,end_s):
        if not math.isfinite(start_s)or not math.isfinite(end_s)or start_s<0 or end_s<=start_s:raise ValueError('Ordered finite nonnegative window required')
        self.clock=clock;self.start_s=start_s;self.base_distance_m=clock.state(start_s)['distance_m']
        self.phase_times=tuple(t-start_s for t in clock.phase_times if start_s<=t<=end_s)
        self.total_join_position_error_m=clock.total_join_position_error_m

    def state(self,time_s):
        s=self.clock.state(time_s+self.start_s).copy();s['distance_m']-=self.base_distance_m;return s


def classify_support_range(pairs,required_range_m,minimum_separation_m,initial_required_range_m):
    """Separate impossible initial conditions, witnessed failure and uncertainty."""
    if not math.isfinite(required_range_m)or required_range_m<=0:raise ValueError('Positive finite support range required')
    if initial_required_range_m>required_range_m:return 'initial_condition_impossible'
    if not pairs:return 'unresolved_bounds'
    if any(p['minimum_distance_witness_m']<minimum_separation_m or p['maximum_distance_witness_m']>required_range_m for p in pairs):return 'witnessed_strategy_violation'
    if all(p['converged']and p['minimum_distance_lower_m']>=minimum_separation_m and p['maximum_distance_upper_m']<=required_range_m for p in pairs):return 'certified_pass'
    return 'unresolved_bounds'
