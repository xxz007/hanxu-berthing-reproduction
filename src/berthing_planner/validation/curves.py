"""Independent circle-centre and phase-integral arithmetic; no production clocks."""
from bisect import bisect_right
import math
import numpy as np



def piece_points(piece, fractions):
    x,y,yaw=piece['pose'];k=piece['curvature'];s=np.asarray(fractions)*piece['length']
    if k==0:return np.column_stack((x+s*np.cos(yaw),y+s*np.sin(yaw)))
    # Rotation of a radius vector about the exact circle centre.
    centre=np.array([x-np.sin(yaw)/k,y+np.cos(yaw)/k])
    angle=yaw+k*s
    return centre+np.column_stack((np.sin(angle)/k,-np.cos(angle)/k))


def speed_checks(profile, pieces, config):
    errors=[];time=distance=0.0
    if len(profile['segments'])!=len(pieces):return ['piece/profile count mismatch']
    for i,(s,p)in enumerate(zip(profile['segments'],pieces)):
        values=[s[k]for k in ('t0_s','t1_s','ramp_duration_s','cruise_duration_s','peak_speed_mps','accel_mps2')]
        if not all(math.isfinite(v)for v in values):errors.append('nonfinite');continue
        ta,tc,v,a=s['ramp_duration_s'],s['cruise_duration_s'],s['peak_speed_mps'],s['accel_mps2']
        expected=p['length'];duration=2*ta+tc
        if min(ta,v,a)<=0 or tc<0:errors.append(f'nonpositive phase {i}')
        if abs(v-a*ta)>1e-8 or abs(v*ta+v*tc-expected)>1e-6:errors.append(f'phase integral {i}')
        if abs(s['t0_s']-time)>1e-7 or abs(s['t1_s']-time-duration)>1e-7:errors.append(f'clock {i}')
        if abs(s['s0_m']-distance)>1e-6:errors.append(f'arc length origin {i}')
        if v>config['cruise_mps']+1e-9 or a>config['tangential_accel_mps2']+1e-9:errors.append(f'kinematic limit {i}')
        if v*v*abs(p['curvature'])>config['lateral_accel_mps2']+1e-9:errors.append(f'lateral limit {i}')
        if abs(s['length_m']-expected)>1e-6:errors.append(f'length {i}')
        time+=duration;distance+=expected
    if abs(time-profile['duration_s'])>1e-7 or abs(distance-profile['length_m'])>1e-6:errors.append('total clock/length')
    return errors


def arc_enclosure(pieces,error=.01):
    points=[];bound=0.0
    for p in pieces:
        k=abs(p['curvature'])
        steps=max(1,math.ceil(p['length']*k/(2*math.acos(1-error*k))))if k else 1
        xy=piece_points(p,np.linspace(0,1,steps+1))
        points.extend(xy.tolist()if not points else xy[1:].tolist())
        if k:bound=max(bound,(1-math.cos(p['length']*k/steps/2))/k)
    return points,bound


class IndependentClock:
    def __init__(self,platform):
        self.pieces=platform['pieces'];self.profile=platform['profile'];self.departure=platform.get('departure_s',0.);self.finish=self.departure+self.profile['duration_s']
        self.starts=[s['t0_s']for s in self.profile['segments']];self.initial=platform.get('initial_pose',self.pieces[0]['pose']if self.pieces else None)
        self.join_error=sum(float(np.linalg.norm(piece_points(a,[1])[0]-b['pose'][:2]))for a,b in zip(self.pieces,self.pieces[1:]))

    def state(self,time):
        local=time-self.departure
        if not self.pieces:return np.asarray(self.initial[:2]),0.,0.,self.initial[2]
        if local<=0:return np.asarray(self.pieces[0]['pose'][:2]),0.,0.,self.pieces[0]['pose'][2]
        if local>=self.profile['duration_s']:
            p=self.pieces[-1];return piece_points(p,[1])[0],self.profile['length_m'],0.,p['pose'][2]+p['curvature']*p['length']
        i=max(0,bisect_right(self.starts,local)-1);s=self.profile['segments'][i];p=self.pieces[i];t=local-s['t0_s'];ta,tc,a,v=s['ramp_duration_s'],s['cruise_duration_s'],s['accel_mps2'],s['peak_speed_mps']
        accelerating=min(t,ta);cruising=min(max(t-ta,0.),tc);decelerating=min(max(t-ta-tc,0.),ta)
        travel=.5*a*accelerating**2+v*cruising+v*decelerating-.5*a*decelerating**2;speed=a*accelerating-a*decelerating
        return piece_points(p,[travel/p['length']])[0],s['s0_m']+travel,speed,p['pose'][2]+p['curvature']*travel


def audit_partition(report,first,second,end):
    errors=[];prior=0.;low=math.inf;high=-math.inf;maximum_radius_error=0.;margin=report['numerical_margin_m']
    for row in report['partition']:
        a,b,m=row['t0_s'],row['t1_s'],row['midpoint_s']
        if abs(a-prior)>1e-7 or b<=a or abs(m-(a+b)/2)>1e-8:errors.append('partition gap/order')
        left=[clock.state(a)for clock in(first,second)];middle=[clock.state(m)for clock in(first,second)];right=[clock.state(b)for clock in(first,second)]
        distance=float(np.linalg.norm(middle[0][0]-middle[1][0]));radius=max(sum(mid[1]-lo[1]for mid,lo in zip(middle,left)),sum(hi[1]-mid[1]for hi,mid in zip(right,middle)),0.)+margin+first.join_error+second.join_error
        maximum_radius_error=max(maximum_radius_error,abs(radius-row['relative_movement_bound_m']))
        if abs(distance-row['midpoint_distance_m'])>1e-6 or abs(radius-row['relative_movement_bound_m'])>1e-6:errors.append('independent midpoint/radius mismatch')
        if row['minimum_distance_lower_m']>max(0.,distance-radius)+1e-7 or row['maximum_distance_upper_m']<distance+radius-1e-7:errors.append('unsafe interval bound')
        low=min(low,row['minimum_distance_lower_m']);high=max(high,row['maximum_distance_upper_m']);prior=b
    if abs(prior-end)>1e-7 or not report['partition']:errors.append('partition horizon')
    for key,timekey in [('minimum_distance_witness_m','minimum_witness_time_s'),('maximum_distance_witness_m','maximum_witness_time_s')]:
        t=report[timekey]
        if not 0<=t<=end:errors.append('witness outside horizon');continue
        actual=float(np.linalg.norm(first.state(t)[0]-second.state(t)[0]))
        if abs(actual-report[key])>1e-6:errors.append('witness mismatch')
    if abs(low-report['minimum_distance_lower_m'])>1e-7 or abs(high-report['maximum_distance_upper_m'])>1e-7:errors.append('global interval extrema')
    if report['converged']and max(report['minimum_distance_witness_m']-low,high-report['maximum_distance_witness_m'])>report['tolerance_m']+1e-6:errors.append('unmet convergence tolerance')
    return {'passed':not errors,'errors':errors,'interval_count':len(report['partition']),'maximum_independent_radius_error_m':maximum_radius_error}


class IndependentSchedule:
    def __init__(self,legs,initial):
        self.initial=initial;self.clocks=[IndependentClock(p)for p in legs];self.join_error=0.;prior=np.asarray(initial[:2])
        for c in self.clocks:
            self.join_error+=c.join_error+float(np.linalg.norm(prior-c.state(c.departure)[0]));prior=c.state(c.finish)[0]

    def state(self,t):
        progress=0.;last=(np.asarray(self.initial[:2]),0.,0.,self.initial[2])
        for c in self.clocks:
            if t<c.departure:break
            p,s,v,yaw=c.state(t);last=(p,progress+s,v,yaw)
            if t<c.finish:break
            progress+=c.profile['length_m']
        return last


class IndependentWindow:
    def __init__(self,clock,start):self.clock=clock;self.start=start;self.base=clock.state(start)[1];self.join_error=clock.join_error
    def state(self,t):
        p,s,v,yaw=self.clock.state(t+self.start);return p,s-self.base,v,yaw
